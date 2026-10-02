#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Restored your original tuned values
static const float ALPHA_ACCEL = 0.65f; 
static const float MAG_NOISE_GATE = 0.01f; 

static float smoothed_gyro[3] = {0.0f, 0.0f, 0.0f};

static float last_accel[3] = {0.0f, 0.0f, 9.81f};
static float last_mag[3] = {0.0f, 1.0f, 0.0f};
static int64_t last_timestamp = 0;

// Replaced 3D states with absolute angles to prevent snap-back
static float last_pitch = 0.0f;
static float last_roll = 0.0f;
static float last_yaw = 0.0f; 

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

// Mathematical protector against 360-degree boundary snap-backs
float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void compute_sensor_fusion(int64_t timestamp) {
    float ax = last_accel[0]; float ay = last_accel[1]; float az = last_accel[2];
    float mx = last_mag[0]; float my = last_mag[1]; float mz = last_mag[2];

    float G = sqrt(ax*ax + ay*ay + az*az);
    if (G < 0.1f) G = 0.1f; 
    float gx = ax / G; float gy = ay / G; float gz = az / G;

    // FIX: Added 0.01f epsilon to prevent atan2(0,0) Gimbal Lock stutter
    float pitch = atan2(ay, az + ((az >= 0.0f) ? 0.01f : -0.01f)); 

    float normalized_ax = -ax / G;
    if (normalized_ax > 1.0f) normalized_ax = 1.0f;
    if (normalized_ax < -1.0f) normalized_ax = -1.0f;
    float roll = asin(normalized_ax); 

    // Extract horizontal magnetic vector
    float dot_mg = mx * gx + my * gy + mz * gz;
    float hx = mx - dot_mg * gx;
    float hy = my - dot_mg * gy;
    float hz = mz - dot_mg * gz;
    float H = sqrt(hx*hx + hy*hy + hz*hz);
    if (H < 0.01f) H = 0.01f;
    hx /= H; hy /= H; hz /= H; 

    // FIX: Absolute 2D Basis Yaw (Replaces glitchy acos cross-product)
    float tx, ty, tz;
    if (fabs(gx) < 0.707f) { tx = 0.0f; ty = gz; tz = -gy; } 
    else { tx = -gz; ty = 0.0f; tz = gx; }
    float normT = sqrt(tx*tx + ty*ty + tz*tz);
    tx /= normT; ty /= normT; tz /= normT;

    float bx = gy * tz - gz * ty;
    float by = gz * tx - gx * tz;
    float bz = gx * ty - gy * tx;

    float mag_u = hx * tx + hy * ty + hz * tz;
    float mag_v = hx * bx + hy * by + hz * bz;
    float yaw = atan2(mag_v, mag_u);

    if (last_timestamp == 0) {
        last_timestamp = timestamp;
        last_pitch = pitch; last_roll = roll; last_yaw = yaw;
        return;
    }

    float raw_dt = (timestamp - last_timestamp) / 1000000000.0f; 
    if (raw_dt <= 0.0f || raw_dt > 0.1f) {
        last_timestamp = timestamp;
        return;
    }

    // FIX: normalize_angle perfectly absorbs the boundary wraps
    float speed_pitch = normalize_angle(pitch - last_pitch) / raw_dt;
    float speed_roll = normalize_angle(roll - last_roll) / raw_dt;
    float speed_yaw = normalize_angle(yaw - last_yaw) / raw_dt;

    if (fabs(speed_yaw) < MAG_NOISE_GATE) speed_yaw = 0.0f;

    // FIX: Universal Verticality Ratio (Fixes 90-degree Landscape deadzone)
    float vertical_ratio = 1.0f - (gz * gz);
    float fade_factor = pow(vertical_ratio, 2.0f);
    
    float final_horizontal_speed = (speed_roll * (1.0f - fade_factor)) + (speed_yaw * fade_factor);

    // Restored YOUR exact output mappings
    smoothed_gyro[0] = ALPHA_ACCEL * speed_pitch + (1.0f - ALPHA_ACCEL) * smoothed_gyro[0];
    smoothed_gyro[1] = ALPHA_ACCEL * final_horizontal_speed + (1.0f - ALPHA_ACCEL) * smoothed_gyro[1];
    smoothed_gyro[2] = 0.0f;

    last_pitch = pitch; last_roll = roll; last_yaw = yaw;
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
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) { 
                last_mag[0] = events[i].magnetic.v[0]; 
                last_mag[1] = events[i].magnetic.v[1]; 
                last_mag[2] = events[i].magnetic.v[2]; 
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
            }
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE) {
                events[i].vector.x = smoothed_gyro[0]; 
                events[i].vector.y = smoothed_gyro[1]; 
                events[i].vector.z = smoothed_gyro[2]; 
            }
        }
        if (fusion_needed && latest_ts > 0) compute_sensor_fusion(latest_ts);
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
