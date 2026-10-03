#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE MIDDLEMAN PARAMETERS ---
static const float ANTI_LAG_BOOST = 1.2f; 
static const float MTK_DEADZONE_THRESHOLD = 0.15f; 
// Your exact proposed baseline, kept just under the engine threshold
static const float BASELINE_BIAS = 0.149999f; 

static float output_gyro[3] = {0.0f, 0.0f, 0.0f};
static float last_stock_gyro[3] = {0.0f, 0.0f, 0.0f};
static int64_t last_timestamp = 0;

// Memory variables to track the last known direction of each axis
static float last_sign_x = 1.0f;
static float last_sign_y = 1.0f;
static float last_sign_z = 1.0f;

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

    // --- 2. DIRECTIONAL MEMORY TRACKING ---
    // Update the memory sign if there is active directional movement
    if (boosted_x > 0.000001f) last_sign_x = 1.0f;
    else if (boosted_x < -0.000001f) last_sign_x = -1.0f;

    if (boosted_y > 0.000001f) last_sign_y = 1.0f;
    else if (boosted_y < -0.000001f) last_sign_y = -1.0f;

    if (boosted_z > 0.000001f) last_sign_z = 1.0f;
    else if (boosted_z < -0.000001f) last_sign_z = -1.0f;

    // --- 3. DYNAMIC BASELINE INJECTION ---
    // If movement is trapped inside the deadzone, inject the directional baseline bias
    if (fabs(boosted_x) < MTK_DEADZONE_THRESHOLD) {
        boosted_x = (last_sign_x * BASELINE_BIAS) + (boosted_x * 2.0f);
    }
    if (fabs(boosted_y) < MTK_DEADZONE_THRESHOLD) {
        boosted_y = (last_sign_y * BASELINE_BIAS) + (boosted_y * 2.0f);
    }
    if (fabs(boosted_z) < MTK_DEADZONE_THRESHOLD) {
        boosted_z = (last_sign_z * BASELINE_BIAS) + (boosted_z * 2.0f);
    }

    // Direct hardware-to-engine output
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
