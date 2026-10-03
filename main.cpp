#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
static const float MTK_DEADZONE = 0.15f; 
static const float LAG_COMPENSATION = 2.5f;

// 1. Fixes the "slow" heavy feeling of the stock vgyro macro-movements
// 2.0f means your physical sweeps are instantly doubled in speed before hitting the engine.
static const float MACRO_SENSITIVITY = 2.0f; 

// 2. Multiplier to match our raw Accel/Mag micro-derivatives to the game's expected sensitivity
static const float MICRO_AIM_SENSITIVITY = 1.5f; 

// Memory for raw sensors
static float last_acc[3] = {0.0f, 0.0f, 0.0f};
static float last_mag[3] = {0.0f, 0.0f, 0.0f};
static int64_t last_acc_ts = 0;
static int64_t last_mag_ts = 0;

// Memory for Phase-Lead (Macro movements)
static float last_vgyro[3] = {0.0f, 0.0f, 0.0f};
static float last_vgyro_delta[3] = {0.0f, 0.0f, 0.0f};

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

// Calculates the raw speed of the physical sensors
float get_sensor_derivative(float current, float &last, int64_t current_ts, int64_t &last_ts) {
    if (last_ts == 0) {
        last = current; last_ts = current_ts; return 0.0f;
    }
    float dt = (current_ts - last_ts) / 1000000000.0f;
    if (dt <= 0.001f || dt > 0.1f) return 0.0f;
    
    float speed = (current - last) / dt;
    last = current; last_ts = current_ts;
    return speed;
}

// Processes the final output sent to Call of Duty
float splice_axis(float raw_vgyro_val, float raw_fallback_speed, float &last_vgyro_val, float &last_delta) {
    float abs_vgyro = fabs(raw_vgyro_val);
    
    // 1. THE GAP FILLER (Micro-Aiming)
    // If the SCP deadzone ate the movement (outputs exactly 0.0 or near zero)
    if (abs_vgyro < 0.01f) {
        last_vgyro_val = 0.0f; 
        last_delta = 0.0f;
        // Inject our raw hardware derivative, scaled to bypass the game's internal deadzone
        if (fabs(raw_fallback_speed) > 0.05f) { // Noise floor for raw hardware static
            float sign = (raw_fallback_speed > 0.0f) ? 1.0f : -1.0f;
            return (sign * MTK_DEADZONE) + (raw_fallback_speed * MICRO_AIM_SENSITIVITY);
        }
        return 0.0f;
    }
    
    // 2. FIX SLOW TRACKING (Scale the stock data up)
    float scaled_vgyro = raw_vgyro_val * MACRO_SENSITIVITY;
    
    // 3. THE PHASE-LEAD BRAKE (Macro-Aiming / Flicks)
    float current_delta = scaled_vgyro - last_vgyro_val;
    float dynamic_boost = LAG_COMPENSATION;
    
    // If decelerating, kill the anti-lag boost so the crosshair stops dead
    if (fabs(current_delta) < fabs(last_delta)) {
        dynamic_boost = 0.0f; 
    }
    
    float predicted_val = scaled_vgyro + (current_delta * dynamic_boost);
    
    // Update memory state using the scaled values
    last_vgyro_val = scaled_vgyro; 
    last_delta = current_delta;
    
    return predicted_val;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        for (ssize_t i = 0; i < actual_events; i++) {
            
            // Track Raw Accelerometer (For Pitch / Vertical Micro-Aim)
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                get_sensor_derivative(events[i].vector.x, last_acc[0], events[i].timestamp, last_acc_ts);
                get_sensor_derivative(events[i].vector.y, last_acc[1], events[i].timestamp, last_acc_ts);
                get_sensor_derivative(events[i].vector.z, last_acc[2], events[i].timestamp, last_acc_ts);
            }
            // Track Raw Compass (For Yaw / Horizontal Micro-Aim)
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD || events[i].type == 14) {
                get_sensor_derivative(events[i].vector.x, last_mag[0], events[i].timestamp, last_mag_ts);
                get_sensor_derivative(events[i].vector.y, last_mag[1], events[i].timestamp, last_mag_ts);
                get_sensor_derivative(events[i].vector.z, last_mag[2], events[i].timestamp, last_mag_ts);
            }
            // Intercept and Splice the Virtual Gyro
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE) {
                
                // Assuming standard Landscape mode mapping. If Pitch/Yaw micro-aim is swapped, switch x/y here.
                float pitch_fallback = get_sensor_derivative(events[i].vector.x, last_acc[0], events[i].timestamp, last_acc_ts); 
                float yaw_fallback = get_sensor_derivative(events[i].vector.y, last_mag[1], events[i].timestamp, last_mag_ts);   
                
                events[i].vector.x = splice_axis(events[i].vector.x, pitch_fallback, last_vgyro[0], last_vgyro_delta[0]);
                events[i].vector.y = splice_axis(events[i].vector.y, yaw_fallback, last_vgyro[1], last_vgyro_delta[1]);
                events[i].vector.z = splice_axis(events[i].vector.z, 0.0f, last_vgyro[2], last_vgyro_delta[2]); 
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
