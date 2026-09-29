#include <android/sensor.h>
#include <android/log.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

#define TAG "NativeGyro"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)

static const float ALPHA = 0.90f; 

// Increased threshold to aggressively kill resting drift (0.030f rad/s)
static const float STATIONARY_THRESHOLD = 0.030f; 

static float smoothed_gyro[3] = {0.0f, 0.0f, 0.0f};
static int64_t last_timestamp = 0;
static float last_pitch = 0.0f;
static float last_roll = 0.0f;

// Hook 1: Data Interceptor
typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;

// Hook 2: Speed Controller
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

void compute_gyro_from_accel(const float current_accel[3], int64_t timestamp, float out_gyro[3]) {
    float x = current_accel[0];
    float y = current_accel[1];
    float z = current_accel[2];

    float pitch = atan2(y, sqrt(x*x + z*z)); 
    float roll = atan2(-x, z); 

    if (last_timestamp == 0) {
        last_timestamp = timestamp;
        last_pitch = pitch;
        last_roll = roll;
        return;
    }

    // Using raw time delta directly; moving average removed
    float raw_dt = (timestamp - last_timestamp) / 1000000000.0f; 
    
    if (raw_dt <= 0.0f || raw_dt > 0.1f) {
        last_timestamp = timestamp;
        return;
    }

    float delta_pitch = (pitch - last_pitch) / raw_dt;
    float delta_roll = (roll - last_roll) / raw_dt;

    // STATIONARY NOISE GATE
    if (fabs(delta_pitch) < STATIONARY_THRESHOLD) delta_pitch = 0.0f;
    if (fabs(delta_roll) < STATIONARY_THRESHOLD) delta_roll = 0.0f;

    smoothed_gyro[0] = ALPHA * delta_pitch + (1.0f - ALPHA) * smoothed_gyro[0];
    smoothed_gyro[1] = ALPHA * delta_roll + (1.0f - ALPHA) * smoothed_gyro[1];
    smoothed_gyro[2] = 0.0f; 

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
                events[i].vector.x = smoothed_gyro[0]; 
                events[i].vector.y = smoothed_gyro[1]; 
                events[i].vector.z = smoothed_gyro[2];
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
        if (target_get) {
            DobbyHook(target_get, 
                     (dobby_dummy_func_t)hook_ASensorEventQueue_getEvents, 
                     (dobby_dummy_func_t*)&orig_getEvents);
        }
        
        void* target_rate = dlsym(libandroid, "ASensorEventQueue_setEventRate");
        if (target_rate) {
            DobbyHook(target_rate, 
                     (dobby_dummy_func_t)hook_ASensorEventQueue_setEventRate, 
                     (dobby_dummy_func_t*)&orig_setEventRate);
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
