#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// --- Performance Tuning ---
static const float ALPHA_ACCEL = 0.75f; 
static const float MAG_NOISE_GATE = 0.015f; 

static float smoothed_gyro[3] = {0.0f, 0.0f, 0.0f};

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
    
    // 90-degree vector lock (prevents division by zero stutter)
    if (H < 0.001f) {
        hx = last_hx; hy = last_hy; hz = last_hz; 
    } else {
        hx /= H; hy /= H; hz /= H; 
    }

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

    // Calculate raw speeds
    float speed_pitch = normalize_angle(pitch - last_pitch) / raw_dt;
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

    // Landscape blend
    float fade_factor = pow(fabs(sin(pitch)), 4.0f);
    float final_horizontal_speed = (speed_roll * (1.0f - fade_factor)) + (speed_yaw * fade_factor);

    // --- ADAPTIVE SMOOTHING (The Zero-Delay Fix) ---
    float adaptive_alpha = ALPHA_ACCEL;
    float abs_horizontal_speed = fabs(final_horizontal_speed);
    float abs_pitch_speed = fabs(speed_pitch);

    // If swiping fast, turn off the filter completely (Instant 1:1 hardware response)
    if (abs_horizontal_speed > 1.2f || abs_pitch_speed > 1.2f) {
        adaptive_alpha = 1.0f; 
    } 
    // If making tiny micro-adjustments or holding still, increase filter to kill jitter
    else if (abs_horizontal_speed < 0.3f && abs_pitch_speed < 0.3f) {
        adaptive_alpha = 0.45f; 
    }

    // Apply the dynamic filter
    smoothed_gyro[0] = adaptive_alpha * speed_pitch + (1.0f - adaptive_alpha) * smoothed_gyro[0];
    smoothed_gyro[1] = adaptive_alpha * final_horizontal_speed + (1.0f - adaptive_alpha) * smoothed_gyro[1];
    smoothed_gyro[2] = 0.0f;

    last_pitch = pitch; last_roll = roll;
    last_hx = hx; last_hy = hy; last_hz = hz;
    last_timestamp = timestamp;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        bool fusion_needed = false;
        int64_t latest_ts = 0;

        for (ssize_t i = 0; i < actual_events; i++) {
            
            // ACCELEROMETER
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) { 
                last_accel[0] = events[i].acceleration.v[0];
                last_accel[1] = events[i].acceleration.v[1];
                last_accel[2] = events[i].acceleration.v[2];
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
            } 
            else if (events[i].type == 35) { 
                last_accel[0] = events[i].data[0];
                last_accel[1] = events[i].data[1];
                last_accel[2] = events[i].data[2];
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
            }
            
            // MAGNETOMETER 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) { 
                last_mag[0] = events[i].magnetic.v[0]; 
                last_mag[1] = events[i].magnetic.v[1]; 
                last_mag[2] = events[i].magnetic.v[2]; 
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
            }
            else if (events[i].type == 14) { 
                last_mag[0] = events[i].data[0]; 
                last_mag[1] = events[i].data[1]; 
                last_mag[2] = events[i].data[2]; 
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
            }
            
            // GYROSCOPE OUTPUT
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
