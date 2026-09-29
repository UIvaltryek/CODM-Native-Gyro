#include <android/sensor.h>
#include <android/log.h>
#include <dlfcn.h>
#include <string.h>
#include "dobby.h"
#include "zygisk.hpp"

#define TAG "NativeGyro"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)

static const float ALPHA = 0.99f;
static float last_accel[3] = {0.0f, 0.0f, 0.0f};
static float smoothed_gyro[3] = {0.0f, 0.0f, 0.0f};
static int64_t last_timestamp = 0;

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;

#include <math.h> // Required for atan2 and sqrt

// Lowering ALPHA to 0.15f (15% raw, 85% history) eliminates the high-frequency jitter
static const float ALPHA = 0.15f; 
static float smoothed_gyro[3] = {0.0f, 0.0f, 0.0f};
static int64_t last_timestamp = 0;
static float last_pitch = 0.0f;
static float last_roll = 0.0f;

void compute_gyro_from_accel(const float current_accel[3], int64_t timestamp, float out_gyro[3]) {
    // Calculate true absolute angles using the gravity vector
    float x = current_accel[0];
    float y = current_accel[1];
    float z = current_accel[2];

    // atan2 converts the 3D gravity projection into precise rotational radians
    float pitch = atan2(y, sqrt(x*x + z*z)); 
    float roll = atan2(-x, z); 

    if (last_timestamp == 0) {
        last_timestamp = timestamp;
        last_pitch = pitch;
        last_roll = roll;
        return;
    }

    float dt = (timestamp - last_timestamp) / 1000000000.0f; 
    if (dt <= 0.0f) return;

    // True Angular Velocity (Rads/sec)
    float delta_pitch = (pitch - last_pitch) / dt;
    float delta_roll = (roll - last_roll) / dt;

    // Apply Heavy Exponential Moving Average to kill screen-tapping jitter
    smoothed_gyro[0] = ALPHA * delta_pitch + (1.0f - ALPHA) * smoothed_gyro[0];
    smoothed_gyro[1] = ALPHA * delta_roll + (1.0f - ALPHA) * smoothed_gyro[1];
    smoothed_gyro[2] = 0.0f; // Z-axis twist cannot be derived from gravity

    out_gyro[0] = smoothed_gyro[0];
    out_gyro[1] = smoothed_gyro[1];
    out_gyro[2] = smoothed_gyro[2];

    last_pitch = pitch;
    last_roll = roll;
    last_timestamp = timestamp;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        for (ssize_t i = 0; i < actual_events; i++) {
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                float synthetic[3];
                compute_gyro_from_accel(events[i].acceleration.v, events[i].timestamp, synthetic);
                        } else if (events[i].type == ASENSOR_TYPE_GYROSCOPE) {
                // Landscape Mode Axis Remapping
                // Swap the physical axes to match the game's rotated screen
                events[i].vector.x = smoothed_gyro[1]; // Feed Y into X
                events[i].vector.y = smoothed_gyro[0]; // Feed X into Y
                events[i].vector.z = smoothed_gyro[2];
            }

        }
    }
    return actual_events;
}

void install_hook() {
    void* libandroid = dlopen("libandroid.so", RTLD_NOW);
    if (libandroid) {
        void* target = dlsym(libandroid, "ASensorEventQueue_getEvents");
        if (target) {
            // FIX: Updated casting to match LSPosed Dobby's strict dobby_dummy_func_t requirement
            DobbyHook(target, 
                     (dobby_dummy_func_t)hook_ASensorEventQueue_getEvents, 
                     (dobby_dummy_func_t*)&orig_getEvents);
            LOGI("ASensorEventQueue_getEvents successfully hooked");
        }
    }
}

class GyroModifier : public zygisk::ModuleBase {
public:
    void onLoad(zygisk::Api* api, JNIEnv* env) override {
        this->api = api;
        this->env = env;
    }

    void preAppSpecialize(zygisk::AppSpecializeArgs* args) override {
        const char* process = env->GetStringUTFChars(args->nice_name, nullptr);
        if (process && strcmp(process, "com.activision.callofduty.shooter") == 0) {
            enable_hack = true;
        }
        env->ReleaseStringUTFChars(args->nice_name, process);
    }

    void postAppSpecialize(const zygisk::AppSpecializeArgs*) override {
        if (enable_hack) {
            install_hook();
        }
    }

private:
    zygisk::Api* api = nullptr;
    JNIEnv* env = nullptr;
    bool enable_hack = false;
};

REGISTER_ZYGISK_MODULE(GyroModifier)
