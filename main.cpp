#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// Production Build: All logging and filters removed for raw 1:1 input
static const float MAG_NOISE_GATE = 0.015f; 

// --- FACTORY BIASES (MT6835 Hard-Iron Offsets) ---
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

static float final_gyro[3] = {0.0f, 0.0f, 0.0f}; 

// Sensor States
static float last_accel[3] = {0.0f, 0.0f, 9.81f}; 
static float last_mag[3] = {0.0f, 1.0f, 0.0f}; 
static int64_t last_timestamp = 0; 

static float last_pitch = 0.0f; 
static float last_roll = 0.0f; 
static float last_hx = 0.0f; 
static float last_hy = 1.0f; 
static float last_hz = 0.0f; 

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t); 
static getEvents_t orig_getEvents = nullptr; 
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

void compute_sensor_fusion(int64_t timestamp) {
    float ax = last_accel[0]; float ay = last_accel[1]; float az = last_accel[2]; 
    float mx = last_mag[0]; float my = last_mag[1]; float mz = last_mag[2]; 

    float G = sqrt(ax*ax + ay*ay + az*az); 
    if (G < 0.1f) G = 0.1f; 

    float gx = ax / G; float gy = ay / G; float gz = az / G; 

    float pitch = atan2(ay, az); 

    float normalized_ax = -ax / G; 
    if (normalized_ax > 1.0f) normalized_ax = 1.0f; 
    if (normalized_ax < -1.0f) normalized_ax = -1.0f; 
    float roll = asin(normalized_ax); 

    float dot_mg = mx * gx + my * gy + mz * gz; 
    float hx = mx - dot_mg * gx; 
    float hy = my - dot_mg * gy; 
    float hz = mz - dot_mg * gz; 

    float H = sqrt(hx*hx + hy*hy + hz*hz); 
    if (H < 0.01f) H = 0.01f; 
    hx /= H; hy /= H; hz /= H; 

    if (last_timestamp == 0) { 
        last_timestamp = timestamp; 
        last_pitch = pitch; last_roll = roll; 
        last_hx = hx; last_hy = hy; last_hz = hz; 
        return; 
    }

    float raw_dt = (timestamp - last_timestamp) / 1000000000.0f; 
    if (raw_dt <= 0.0f || raw_dt > 0.1f) { 
        last_timestamp = timestamp; 
        return; 
    }

    float speed_pitch = (pitch - last_pitch) / raw_dt; 
    float speed_roll = (roll - last_roll) / raw_dt; 

    float dot_h = hx * last_hx + hy * last_hy + hz * last_hz; 
    if (dot_h > 1.0f) dot_h = 1.0f; 
    if (dot_h < -1.0f) dot_h = -1.0f; 
    float mag_delta_yaw = acos(dot_h); 

    float cx = last_hy * hz - last_hz * hy; 
    float cy = last_hz * hx - last_hx * hz; 
    float cz = last_hx * hy - last_hy * hx; 
    
    float direction = cx * gx + cy * gy + cz * gz; 
    if (direction < 0.0f) mag_delta_yaw = -mag_delta_yaw; 

    float speed_yaw = mag_delta_yaw / raw_dt; 
    if (fabs(speed_yaw) < MAG_NOISE_GATE) speed_yaw = 0.0f; 

    float fade_factor = pow(fabs(sin(pitch)), 4.0f); 
    
    float final_horizontal_speed = (speed_roll * (1.0f - fade_factor)) + (speed_yaw * fade_factor); 

    // DIRECT FEED (No ALPHA_ACCEL Low-Pass Filter)
    final_gyro[0] = speed_pitch; 
    final_gyro[1] = final_horizontal_speed; 
    final_gyro[2] = 0.0f; 

    last_pitch = pitch; last_roll = roll; 
    last_hx = hx; last_hy = hy; last_hz = hz; 
    last_timestamp = timestamp; 
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        bool fusion_ready = false;
        int64_t latest_ts = 0;

        for (ssize_t i = 0; i < actual_events; i++) {
            latest_ts = events[i].timestamp;
            
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                last_accel[0] = events[i].acceleration.x;
                last_accel[1] = events[i].acceleration.y;
                last_accel[2] = events[i].acceleration.z;
                fusion_ready = true;
            } 
            // STRICTLY UNCALIBRATED MAGNETOMETER WITH HARD-IRON OFFSETS
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                last_mag[0] = events[i].uncalibrated_magnetic.x_uncalib - HARD_IRON_X;
                last_mag[1] = events[i].uncalibrated_magnetic.y_uncalib - HARD_IRON_Y;
                last_mag[2] = events[i].uncalibrated_magnetic.z_uncalib - HARD_IRON_Z;
                fusion_ready = true;
            }
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE || events[i].type == 4 || events[i].type == 16) {
                events[i].vector.x = final_gyro[0]; 
                events[i].vector.y = final_gyro[1]; 
                events[i].vector.z = final_gyro[2]; 
            }
        }

        if (fusion_ready && latest_ts > 0) {
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
