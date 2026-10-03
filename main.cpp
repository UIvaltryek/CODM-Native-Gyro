#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE MIDDLEMAN PARAMETERS ---
// Pushes the signal forward aggressively to cancel MediaTek's low-pass delay
static const float ANTI_LAG_BOOST = 1.2f; 
// The exact mathematical suppression threshold of the stock OS
static const float MTK_DEADZONE_THRESHOLD = 0.15f; 

static float output_gyro[3] = {0.0f, 0.0f, 0.0f};
static float last_stock_gyro[3] = {0.0f, 0.0f, 0.0f};
static int64_t last_timestamp = 0;

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

void reshape_stock_gyro(float stock_x, float stock_y, float stock_z, int64_t timestamp) {
    if (last_timestamp == 0) {
        last_timestamp = timestamp;
        last_stock_gyro[0] = stock_x; last_stock_gyro[1] = stock_y; last_stock_gyro[2] = stock_z;
        return;
    }

    float dt = (timestamp - last_timestamp) / 1000000000.0f; 
    if (dt <= 0.001f || dt > 0.1f) { 
        last_timestamp = timestamp;
        return;
    }

    // --- 1. ANTI-LAG PREDICTIVE BOOST ---
    float accel_x = (stock_x - last_stock_gyro[0]) / dt;
    float accel_y = (stock_y - last_stock_gyro[1]) / dt;
    float accel_z = (stock_z - last_stock_gyro[2]) / dt;

    float boosted_x = stock_x + (accel_x * dt * ANTI_LAG_BOOST);
    float boosted_y = stock_y + (accel_y * dt * ANTI_LAG_BOOST);
    float boosted_z = stock_z + (accel_z * dt * ANTI_LAG_BOOST);

    // --- 2. ANTI-DEADZONE INJECTION (ADZ) ---
    // Floor lowered to 1e-6 to catch heavily suppressed floats before they hit absolute zero
    if (fabs(stock_x) > 0.000001f && fabs(stock_x) < MTK_DEADZONE_THRESHOLD) {
        float sign_x = (stock_x > 0.0f) ? 1.0f : -1.0f;
        boosted_x = (sign_x * MTK_DEADZONE_THRESHOLD) + (stock_x * 2.0f);
    }
    
    if (fabs(stock_y) > 0.000001f && fabs(stock_y) < MTK_DEADZONE_THRESHOLD) {
        float sign_y = (stock_y > 0.0f) ? 1.0f : -1.0f;
        boosted_y = (sign_y * MTK_DEADZONE_THRESHOLD) + (stock_y * 2.0f);
    }
    
    if (fabs(stock_z) > 0.000001f && fabs(stock_z) < MTK_DEADZONE_THRESHOLD) {
        float sign_z = (stock_z > 0.0f) ? 1.0f : -1.0f;
        boosted_z = (sign_z * MTK_DEADZONE_THRESHOLD) + (stock_z * 2.0f);
    }

    // Direct hardware-to-engine output (zero smoothing to eliminate phase lag)
    output_gyro[0] = boosted_x;
    output_gyro[1] = boosted_y;
    output_gyro[2] = boosted_z;

    last_stock_gyro[0] = stock_x; 
    last_stock_gyro[1] = stock_y; 
    last_stock_gyro[2] = stock_z;
    last_timestamp = timestamp;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        for (ssize_t i = 0; i < actual_events; i++) {
            
            // Intercept only the Calibrated Stock Gyroscope (Type 4)
            if (events[i].type == ASENSOR_TYPE_GYROSCOPE) {
                reshape_stock_gyro(events[i].vector.x, events[i].vector.y, events[i].vector.z, events[i].timestamp);
                events[i].vector.x = output_gyro[0]; 
                events[i].vector.y = output_gyro[1]; 
                events[i].vector.z = output_gyro[2]; 
            }
        }
    }
    return actual_events;
}

int hook_ASensorEventQueue_setEventRate(ASensorEventQueue* queue, ASensor const* sensor, int32_t usec) {
    // Override the game's requested delay, force fastest possible polling rate
    return orig_setEventRate(queue, sensor, 0); 
}

void install_hook() {
    void* libandroid = dlopen("libandroid.so", RTLD_NOW);
    if (libandroid) {
        void* target_get = dlsym(libandroid, "ASensorEventQueue_getEvents");
        if (target_get) DobbyHook(target_get, (dobby_dummy_func_t)hook_ASensorEventQueue_getEvents, (dobby_dummy_func_t*)&orig_getEvents);
        
        void* target_rate = dlsym(libandroid, "ASensorEventQueue_setEventRate");
        if (target_rate) DobbyHook(target_rate, (dobby_dummy_func_t)hook_ASensorEventQueue_setEventRate, (dobby_dummy_func_t*)&orig_setEventRate);
    }
}

class GyroModifier : public zygisk::ModuleBase {
public:
    void onLoad(zygisk::Api* api, JNIEnv* env) override { this->api = api; this->env = env; }
    void preAppSpecialize(zygisk::AppSpecializeArgs* args) override {
        const char* process = env->GetStringUTFChars(args->nice_name, nullptr);
        if (process && strcmp(process, "com.activision.callofduty.shooter") == 0) { enable_hack = true; }
        env->ReleaseStringUTFChars(args->nice_name, process);
    }
    void postAppSpecialize(const zygisk::AppSpecializeArgs*) override { if (enable_hack) install_hook(); }
private:
    zygisk::Api* api = nullptr; JNIEnv* env = nullptr; bool enable_hack = false;
};
REGISTER_ZYGISK_MODULE(GyroModifier)
