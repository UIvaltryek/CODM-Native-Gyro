#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
static const float SENSITIVITY = 1.0f; // Global sensitivity multiplier
static const float STATIC_NOISE_GATE = 0.002f; // Microscopic gate purely to kill stationary 200Hz magnetic vibration

// MT6835 Hard-Iron Biases
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

// --- 5-FRAME SLIDING WINDOW (LINEAR REGRESSION) ---
// Smooths out sensor steps without creating "ice-sliding" phase lag
class RegressionBuffer {
public:
    float t[5] = {0};
    float x[5] = {0}, y[5] = {0}, z[5] = {0};
    int index = 0;
    int count = 0;

    void push(float time_sec, float vx, float vy, float vz) {
        t[index] = time_sec; x[index] = vx; y[index] = vy; z[index] = vz;
        index = (index + 1) % 5;
        if (count < 5) count++;
    }

    void derive(float& dx, float& dy, float& dz) {
        if (count < 2) { dx = 0; dy = 0; dz = 0; return; }
        
        float sum_t = 0, sum_x = 0, sum_y = 0, sum_z = 0;
        for (int i = 0; i < count; i++) {
            sum_t += t[i]; sum_x += x[i]; sum_y += y[i]; sum_z += z[i];
        }
        float mean_t = sum_t / count;
        float mean_x = sum_x / count;
        float mean_y = sum_y / count;
        float mean_z = sum_z / count;

        float num_x = 0, num_y = 0, num_z = 0, den = 0;
        for (int i = 0; i < count; i++) {
            float dt = t[i] - mean_t;
            den += dt * dt;
            num_x += dt * (x[i] - mean_x);
            num_y += dt * (y[i] - mean_y);
            num_z += dt * (z[i] - mean_z);
        }
        
        if (den < 1e-6f) { dx = 0; dy = 0; dz = 0; return; } 
        
        dx = num_x / den; 
        dy = num_y / den; 
        dz = num_z / den;
    }
};

static RegressionBuffer accel_buffer;
static RegressionBuffer mag_buffer;

static float current_A[3] = {0.0f, 0.0f, 1.0f};
static float current_M[3] = {0.0f, 1.0f, 0.0f};
static float final_gyro[3] = {0.0f, 0.0f, 0.0f};

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

void normalize(float v[3]) {
    float norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (norm > 0.001f) { v[0] /= norm; v[1] /= norm; v[2] /= norm; }
}

void cross_product(float a[3], float b[3], float out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

void compute_additive_kinematics() {
    float dot_A[3], dot_M[3];
    
    // 1. Get the perfectly smoothed rates of change from the 25ms regression buffers
    accel_buffer.derive(dot_A[0], dot_A[1], dot_A[2]);
    mag_buffer.derive(dot_M[0], dot_M[1], dot_M[2]);

    // 2. The Accelerometer Spin (Dominates Pitch/Roll, drops Yaw at 90 deg)
    float w_acc[3];
    cross_product(current_A, dot_A, w_acc);

    // 3. The Magnetometer Spin (Dominates Yaw, drops Pitch/Roll)
    float w_mag[3];
    cross_product(current_M, dot_M, w_mag);
    
    // 4. Your Masterpiece: Pure Additive Synthesis
    float raw_x = (w_acc[0] + w_mag[0]) * SENSITIVITY;
    float raw_y = (w_acc[1] + w_mag[1]) * SENSITIVITY;
    float raw_z = (w_acc[2] + w_mag[2]) * SENSITIVITY;

    // 5. Apply the raw data with a tiny noise gate to kill table-wobble
    final_gyro[0] = (fabs(raw_x) > STATIC_NOISE_GATE) ? raw_x : 0.0f;
    final_gyro[1] = (fabs(raw_y) > STATIC_NOISE_GATE) ? raw_y : 0.0f;
    final_gyro[2] = (fabs(raw_z) > STATIC_NOISE_GATE) ? raw_z : 0.0f;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        bool fusion_ready = false;

        for (ssize_t i = 0; i < actual_events; i++) {
            float ts_sec = events[i].timestamp / 1000000000.0f;

            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                current_A[0] = events[i].acceleration.x;
                current_A[1] = events[i].acceleration.y;
                current_A[2] = events[i].acceleration.z;
                normalize(current_A);
                accel_buffer.push(ts_sec, current_A[0], current_A[1], current_A[2]);
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                current_M[0] = events[i].uncalibrated_magnetic.x_uncalib - HARD_IRON_X;
                current_M[1] = events[i].uncalibrated_magnetic.y_uncalib - HARD_IRON_Y;
                current_M[2] = events[i].uncalibrated_magnetic.z_uncalib - HARD_IRON_Z;
                normalize(current_M);
                mag_buffer.push(ts_sec, current_M[0], current_M[1], current_M[2]);
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) {
                current_M[0] = events[i].magnetic.x;
                current_M[1] = events[i].magnetic.y;
                current_M[2] = events[i].magnetic.z;
                normalize(current_M);
                mag_buffer.push(ts_sec, current_M[0], current_M[1], current_M[2]);
                fusion_ready = true;
            } 
            // Feed the raw additive angular velocity directly to the game
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE || events[i].type == 4 || events[i].type == 16) {
                events[i].vector.x = final_gyro[0]; 
                events[i].vector.y = final_gyro[1]; 
                events[i].vector.z = final_gyro[2]; 
            }
        }

        if (fusion_ready) {
            compute_additive_kinematics();
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
