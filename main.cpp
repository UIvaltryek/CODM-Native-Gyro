#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// Performance Tuning
// Increased to 0.85f for instantaneous, zero-delay response.
static const float ALPHA_ACCEL = 0.85f; 

static float smoothed_gyro[3] = {0.0f, 0.0f, 0.0f};

// Raw Sensor States
static float last_accel[3] = {0.0f, 0.0f, 9.81f};
static float last_mag[3] = {0.0f, 1.0f, 0.0f};
static int64_t last_timestamp = 0;

// 3D Vector Tracking States (Replaces Pitch/Roll/Yaw Angles)
static float prev_a[3] = {0.0f, 0.0f, 1.0f};
static float prev_h[3] = {0.0f, 1.0f, 0.0f};

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;

typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

// 3D Vector Cross-Product Fusion (Immune to Gimbal Lock and Boundary Snap-Back)
void compute_sensor_fusion(int64_t timestamp) {
    // 1. Normalize current Accelerometer (Gravity Vector)
    float ax = last_accel[0]; float ay = last_accel[1]; float az = last_accel[2];
    float normA = sqrt(ax*ax + ay*ay + az*az);
    if (normA < 0.1f) normA = 0.1f;
    ax /= normA; ay /= normA; az /= normA;

    // 2. Project Magnetometer onto a flat 2D plane perpendicular to Gravity
    float mx = last_mag[0]; float my = last_mag[1]; float mz = last_mag[2];
    float dotMA = mx * ax + my * ay + mz * az;
    float hx = mx - dotMA * ax;
    float hy = my - dotMA * ay;
    float hz = mz - dotMA * az;
    
    float normH = sqrt(hx*hx + hy*hy + hz*hz);
    if (normH < 0.01f) normH = 0.01f;
    hx /= normH; hy /= normH; hz /= normH;

    // First frame initialization
    if (last_timestamp == 0) {
        prev_a[0] = ax; prev_a[1] = ay; prev_a[2] = az;
        prev_h[0] = hx; prev_h[1] = hy; prev_h[2] = hz;
        last_timestamp = timestamp;
        return;
    }

    float dt = (timestamp - last_timestamp) / 1000000000.0f;
    if (dt <= 0.0f || dt > 0.1f) {
        last_timestamp = timestamp;
        return;
    }

    // 3. CROSS PRODUCT 1: Pitch & Roll Velocity
    float cx = prev_a[1] * az - prev_a[2] * ay;
    float cy = prev_a[2] * ax - prev_a[0] * az;
    float cz = prev_a[0] * ay - prev_a[1] * ax;

    // 4. CROSS PRODUCT 2: Yaw Velocity (Rotation around Gravity vector)
    float hx_cross = prev_h[1] * hz - prev_h[2] * hy;
    float hy_cross = prev_h[2] * hx - prev_h[0] * hz;
    float hz_cross = prev_h[0] * hy - prev_h[1] * hx;
    
    // Dot product gives absolute Z-axis rotational speed
    float yaw_sin = hx_cross * ax + hy_cross * ay + hz_cross * az;
    float yaw_speed = yaw_sin / dt; 

    // 5. Final 3D Gyroscope Output (rad/s)
    float raw_gx = (cx / dt) + (yaw_speed * ax);
    float raw_gy = (cy / dt) + (yaw_speed * ay);
    float raw_gz = (cz / dt) + (yaw_speed * az);

    // 6. Apply High-Speed EMA Filter
    smoothed_gyro[0] = ALPHA_ACCEL * raw_gx + (1.0f - ALPHA_ACCEL) * smoothed_gyro[0];
    smoothed_gyro[1] = ALPHA_ACCEL * raw_gy + (1.0f - ALPHA_ACCEL) * smoothed_gyro[1];
    smoothed_gyro[2] = ALPHA_ACCEL * raw_gz + (1.0f - ALPHA_ACCEL) * smoothed_gyro[2];

    // Save vectors for the next frame
    prev_a[0] = ax; prev_a[1] = ay; prev_a[2] = az;
    prev_h[0] = hx; prev_h[1] = hy; prev_h[2] = hz;
    last_timestamp = timestamp;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        bool fusion_needed = false;
        int64_t latest_ts = 0;

        for (ssize_t i = 0; i < actual_events; i++) {
            
            // Standard Calibrated Accelerometer (Type 1)
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) { 
                last_accel[0] = events[i].acceleration.v[0];
                last_accel[1] = events[i].acceleration.v[1];
                last_accel[2] = events[i].acceleration.v[2];
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
            } 
            // Uncalibrated Accelerometer (Type 35) Fallback
            else if (events[i].type == 35) { 
                last_accel[0] = events[i].data[0];
                last_accel[1] = events[i].data[1];
                last_accel[2] = events[i].data[2];
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
            }
            // Standard Calibrated Compass (Type 2)
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) { 
                last_mag[0] = events[i].magnetic.v[0]; 
                last_mag[1] = events[i].magnetic.v[1]; 
                last_mag[2] = events[i].magnetic.v[2]; 
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
            }
            // Uncalibrated Compass (Type 14) Fallback
            else if (events[i].type == 14) { 
                last_mag[0] = events[i].data[0]; 
                last_mag[1] = events[i].data[1]; 
                last_mag[2] = events[i].data[2]; 
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
            }
            // Inject calculated math into Virtual Gyroscope (Type 4)
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

// Universal speed uncap hook (Safety net for pure native C++ engines)
int hook_ASensorEventQueue_setEventRate(ASensorEventQueue* queue, ASensor const* sensor, int32_t usec) {
    return orig_setEventRate(queue, sensor, 0); 
}

void install_hook() {
    void* libandroid = dlopen("libandroid.so", RTLD_NOW);
    if (libandroid) {
        void* target_get = dlsym(libandroid, "ASensorEventQueue_getEvents");
        if (target_get) {
            DobbyHook(target_get, (dobby_dummy_func_t)hook_ASensorEventQueue_getEvents, (dobby_dummy_func_t*)&orig_getEvents);
        }
        void* target_rate = dlsym(libandroid, "ASensorEventQueue_setEventRate");
        if (target_rate) {
            DobbyHook(target_rate, (dobby_dummy_func_t)hook_ASensorEventQueue_setEventRate, (dobby_dummy_func_t*)&orig_setEventRate);
        }
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
