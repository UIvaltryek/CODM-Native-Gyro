#include <android/sensor.h>
#include <android/log.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

#define TAG "NativeGyro"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)

// Accelerometer filter (Fast - 200Hz)
static const float ALPHA_ACCEL = 0.25f; 
// Magnetometer filter (Upgraded to match 100Hz hardware capability)
static const float ALPHA_MAG = 0.20f; 

// Dedicated noise gate to block magnetic static without causing delay
static const float MAG_NOISE_GATE = 0.025f;

static float smoothed_gyro[3] = {0.0f, 0.0f, 0.0f};

// Sensor States
static float last_accel[3] = {0.0f, 0.0f, 9.81f};
static float last_mag[3] = {0.0f, 1.0f, 0.0f};
static int64_t last_timestamp = 0;

static float last_pitch = 0.0f;
static float last_roll = 0.0f;
static float last_yaw = 0.0f;

// Drift Eliminators
static float pitch_bias = 0.0f;
static float roll_bias = 0.0f;
static float yaw_bias = 0.0f;

// Hook 1: Data Interceptor
typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;

// Hook 2: Speed Controller
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

void compute_sensor_fusion(int64_t timestamp) {
    float ax = last_accel[0];
    float ay = last_accel[1];
    float az = last_accel[2];

    float mx = last_mag[0];
    float my = last_mag[1];
    float mz = last_mag[2];

    // 1. Calculate Gravity and Pitch/Roll (Immune to horizontal bleed)
    float G = sqrt(ax*ax + ay*ay + az*az);
    if (G < 0.1f) G = 0.1f; 

    float pitch = atan2(ay, sqrt(ax*ax + az*az)); 
    
    float normalized_ax = -ax / G;
    if (normalized_ax > 1.0f) normalized_ax = 1.0f;
    if (normalized_ax < -1.0f) normalized_ax = -1.0f;
    float roll = asin(normalized_ax); 

    // 2. Tilt-Compensated Magnetometer (Yaw)
    float my_comp = my * cos(roll) - mz * sin(roll);
    float mx_comp = mx * cos(pitch) + my * sin(pitch) * sin(roll) + mz * sin(pitch) * cos(roll);
    float yaw = atan2(my_comp, mx_comp);

    if (last_timestamp == 0) {
        last_timestamp = timestamp;
        last_pitch = pitch;
        last_roll = roll;
        last_yaw = yaw;
        return;
    }

    float raw_dt = (timestamp - last_timestamp) / 1000000000.0f; 
    if (raw_dt <= 0.0f || raw_dt > 0.1f) {
        last_timestamp = timestamp;
        return;
    }

    float delta_pitch = (pitch - last_pitch);
    float delta_roll = (roll - last_roll);
    float delta_yaw = (yaw - last_yaw);

    // 3. Pi-Wrap Correction 
    if (delta_yaw > M_PI) delta_yaw -= 2.0f * M_PI;
    if (delta_yaw < -M_PI) delta_yaw += 2.0f * M_PI;

    // Convert to speeds (rad/s)
    float speed_pitch = delta_pitch / raw_dt;
    float speed_roll = delta_roll / raw_dt;
    float speed_yaw = delta_yaw / raw_dt;

    // 4. Magnetic Noise Gate (Clamps room static to absolute zero)
    if (fabs(speed_yaw) < MAG_NOISE_GATE) speed_yaw = 0.0f;

    // 5. Drift Eliminators (High-Pass Filters)
    pitch_bias = 0.0005f * speed_pitch + 0.9995f * pitch_bias;
    roll_bias = 0.0005f * speed_roll + 0.9995f * roll_bias;
    yaw_bias = 0.0005f * speed_yaw + 0.9995f * yaw_bias;

    speed_pitch -= pitch_bias;
    speed_roll -= roll_bias;
    speed_yaw -= yaw_bias;

    // 6. Apply Split Filters for 200Hz Accel and 100Hz Mag
    smoothed_gyro[0] = ALPHA_ACCEL * speed_pitch + (1.0f - ALPHA_ACCEL) * smoothed_gyro[0];
    smoothed_gyro[1] = ALPHA_ACCEL * speed_roll + (1.0f - ALPHA_ACCEL) * smoothed_gyro[1];
    smoothed_gyro[2] = ALPHA_MAG * speed_yaw + (1.0f - ALPHA_MAG) * smoothed_gyro[2];

    last_pitch = pitch;
    last_roll = roll;
    last_yaw = yaw;
    last_timestamp = timestamp;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        bool fusion_needed = false;
        int64_t latest_ts = 0;

        for (ssize_t i = 0; i < actual_events; i++) {
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                last_accel[0] = events[i].acceleration.v[0];
                last_accel[1] = events[i].acceleration.v[1];
                last_accel[2] = events[i].acceleration.v[2];
                latest_ts = events[i].timestamp;
                fusion_needed = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) {
                last_mag[0] = events[i].magnetic.v[0];
                last_mag[1] = events[i].magnetic.v[1];
                last_mag[2] = events[i].magnetic.v[2];
                latest_ts = events[i].timestamp;
                fusion_needed = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE) {
                events[i].vector.x = smoothed_gyro[0]; 
                events[i].vector.y = smoothed_gyro[1]; 
                events[i].vector.z = smoothed_gyro[2]; 
            }
        }

        if (fusion_needed && latest_ts > 0) {
            compute_sensor_fusion(latest_ts);
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
