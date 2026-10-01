#include <android/sensor.h>
#include <android/log.h>
#include <dlfcn.h>
#include <string.h>
#include "dobby.h"
#include "zygisk.hpp"

#define TAG "NativeGyro_Unlocker"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)

// Hook: Speed Controller Only (No data manipulation)
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

int hook_ASensorEventQueue_setEventRate(ASensorEventQueue* queue, ASensor const* sensor, int32_t usec) {
    // Forces absolute fastest hardware polling (0 microseconds) for ALL sensors
    return orig_setEventRate(queue, sensor, 0); 
}

void install_hook() {
    void* libandroid = dlopen("libandroid.so", RTLD_NOW);
    if (libandroid) {
        // We only hook the rate setter, completely leaving getEvents (the actual data) alone
        void* target_rate = dlsym(libandroid, "ASensorEventQueue_setEventRate");
        if (target_rate) {
            DobbyHook(target_rate, 
                     (dobby_dummy_func_t)hook_ASensorEventQueue_setEventRate, 
                     (dobby_dummy_func_t*)&orig_setEventRate);
            LOGI("Sensor polling rate uncapped successfully.");
        }
    }
}

class GyroUnlocker : public zygisk::ModuleBase {
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

REGISTER_ZYGISK_MODULE(GyroUnlocker)
