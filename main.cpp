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

void compute_gyro_from_accel(const float current_accel[3], int64_t timestamp, float out_gyro[3]) {
    if (last_timestamp == 0) {
        last_timestamp = timestamp;
        last_accel[0] = current_accel[0];
        last_accel[1] = current_accel[1];
        last_accel[2] = current_accel[2];
        return;
    }

    float dt = (timestamp - last_timestamp) / 1000000000.0f; 
    if (dt <= 0.0f) return;

    float delta_x = (current_accel[0] - last_accel[0]) / dt;
    float delta_y = (current_accel[1] - last_accel[1]) / dt;
    float delta_z = (current_accel[2] - last_accel[2]) / dt;

    smoothed_gyro[0] = ALPHA * delta_x + (1.0f - ALPHA) * smoothed_gyro[0];
    smoothed_gyro[1] = ALPHA * delta_y + (1.0f - ALPHA) * smoothed_gyro[1];
    smoothed_gyro[2] = ALPHA * delta_z + (1.0f - ALPHA) * smoothed_gyro[2];

    out_gyro[0] = smoothed_gyro[0];
    out_gyro[1] = smoothed_gyro[1];
    out_gyro[2] = smoothed_gyro[2];

    last_accel[0] = current_accel[0];
    last_accel[1] = current_accel[1];
    last_accel[2] = current_accel[2];
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
