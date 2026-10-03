#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
// 1. Hardware noise limit. Drops static table drift to 0.0.
static const float NOISE_FLOOR = 0.008f; 
// 2. The game engine's suppression wall (ADZ Teleport)
static const float MTK_DEADZONE = 0.15f; 
// 3. Phase-Lead Multiplier. Cancels the "smooth delay" by pushing aim forward.
static const float LAG_COMPENSATION = 2.5f;

// Memory to track the previous frame's position
static float last_raw_x = 0.0f;
static float last_raw_y = 0.0f;
static float last_raw_z = 0.0f;

// Memory to track the previous frame's speed (to detect braking)
static float last_delta_x = 0.0f;
static float last_delta_y = 0.0f;
static float last_delta_z = 0.0f;

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

// Phase-Lead Remap Function with Deceleration Damping (Dynamic Brake)
float process_axis(float current_val, float &last_val, float &last_delta) {
    float abs_val = fabs(current_val);
    
    // 1. Kill the static table drift
    if (abs_val < NOISE_FLOOR) {
        last_val = current_val; 
        last_delta = 0.0f;
        return 0.0f; 
    }
    
    // 2. Calculate current speed (frame-to-frame change)
    float current_delta = current_val - last_val;
    float abs_current_delta = fabs(current_delta);
    float abs_last_delta = fabs(last_delta);
    
    // 3. Dynamic Brake: Are we accelerating or decelerating?
    float dynamic_boost = LAG_COMPENSATION;
    if (abs_current_delta < abs_last_delta) {
        // Hand is slowing down (flick is ending). Kill the boost to prevent overshoot.
        dynamic_boost = 0.0f; 
    }
    
    // Push the output forward using the dynamically braked multiplier
    float predicted_val = current_val + (current_delta * dynamic_boost);
    
    // Save current frame data for the next calculation
    last_val = current_val; 
    last_delta = current_delta;
    
    // 4. Continuous Deadzone Bypass
    float sign = (predicted_val > 0.0f) ? 1.0f : -1.0f;
    return (sign * MTK_DEADZONE) + predicted_val;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        for (ssize_t i = 0; i < actual_events; i++) {
            
            // Intercept only the Calibrated Stock Gyroscope (Type 4)
            if (events[i].type == ASENSOR_TYPE_GYROSCOPE) {
                // Process each axis through the Phase-Lead compensator directly
                events[i].vector.x = process_axis(events[i].vector.x, last_raw_x, last_delta_x);
                events[i].vector.y = process_axis(events[i].vector.y, last_raw_y, last_delta_y);
                events[i].vector.z = process_axis(events[i].vector.z, last_raw_z, last_delta_z);
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
