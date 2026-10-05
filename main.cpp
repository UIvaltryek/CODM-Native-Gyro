#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- MIDDLEMAN TUNABLE PARAMETERS ---

// 1. The Deadzone Spring
static const float NOISE_GATE = 0.005f;      // Ignore autonomous wobbling below this rad/s
static const float BOOST_OFFSET = 0.13999f;  // Your exact pre-tensioning offset
static const float FADE_THRESHOLD = 0.5f;    // Speed (rad/s) where the boost completely fades to 0.0 to preserve fast flicks

// 2. The Inverse Filter (Lead Compensator)
// Represents the time-constant (in seconds) of MediaTek's lag. 
// A value of 0.05f looks 50 milliseconds into the future to cancel the phase lag.
// Increase to 0.10f if the ending "ice-slide" lag is still too long.
static const float SHARPNESS = 0.05f; 

// --- STATE TRACKERS ---
static float last_x = 0.0f;
static float last_y = 0.0f;
static float last_z = 0.0f;
static int64_t last_ts = 0;

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

float process_axis(float current_val, float &last_val, float dt) {
    // 1. DECONVOLUTION (Kill Start/End Lag)
    // Calculate exact acceleration (Derivative)
    float derivative = (current_val - last_val) / dt;
    
    // Add the predictive lead to cancel MediaTek's low-pass buffer
    float sharpened = current_val + (SHARPNESS * derivative);
    last_val = current_val;

    // 2. THE DIGITAL BRAKE & NOISE GATE
    if (fabs(sharpened) < NOISE_GATE) {
        return 0.0f; 
    }

    // 3. DEADZONE SPRING (with Linear Fade)
    float sign = (sharpened > 0.0f) ? 1.0f : -1.0f;
    
    // Calculate how much boost to apply.
    // At 0.005 rad/s, it applies nearly 100% of 0.13999.
    // At >= 0.5 rad/s, the boost drops to exactly 0.0, leaving your fast flicks perfectly natural.
    float fade_factor = fmax(0.0f, 1.0f - (fabs(sharpened) / FADE_THRESHOLD));
    float dynamic_boost = BOOST_OFFSET * fade_factor;

    // Output the lag-free signal + the deadzone spring
    return sharpened + (sign * dynamic_boost);
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        for (ssize_t i = 0; i < actual_events; i++) {
            
            // Intercept standard Gyroscope (Type 4) and Uncalibrated Gyroscope (Type 16)
            if (events[i].type == ASENSOR_TYPE_GYROSCOPE || events[i].type == 16) {
                
                int64_t current_ts = events[i].timestamp;
                if (last_ts == 0) {
                    last_ts = current_ts;
                    last_x = events[i].vector.x;
                    last_y = events[i].vector.y;
                    last_z = events[i].vector.z;
                    continue;
                }

                // Calculate exact delta time in seconds
                float dt = (current_ts - last_ts) / 1000000000.0f;
                if (dt <= 0.0001f || dt > 0.1f) dt = 0.01f; // Fallback for anomalous polling gaps

                float raw_x = events[i].vector.x;
                float raw_y = events[i].vector.y;
                float raw_z = events[i].vector.z;

                // Process axes individually
                events[i].vector.x = process_axis(raw_x, last_x, dt);
                events[i].vector.y = process_axis(raw_y, last_y, dt);
                events[i].vector.z = process_axis(raw_z, last_z, dt);
                
                last_ts = current_ts;
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
        if (process && strcmp(process, "com.activision.callofduty.shooter") == 0) { 
            enable_hack = true; 
        }
        env->ReleaseStringUTFChars(args->nice_name, process);
    }
    void postAppSpecialize(const zygisk::AppSpecializeArgs*) override { 
        if (enable_hack) install_hook(); 
    }
private:
    zygisk::Api* api = nullptr; 
    JNIEnv* env = nullptr; 
    bool enable_hack = false;
};
REGISTER_ZYGISK_MODULE(GyroModifier)
