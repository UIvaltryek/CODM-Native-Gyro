#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
static const float MAG_NOISE_GATE = 0.005f; 
static const float SENSITIVITY = 1.0f;

// Factory Bias for MT6835 - Permanently centers the origin[span_0](start_span)[span_0](end_span)
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

struct Vec3 { float x, y, z; };

Vec3 cross(Vec3 a, Vec3 b) {
    return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}
float dot(Vec3 a, Vec3 b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
float len(Vec3 v) {
    return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}
Vec3 normalize(Vec3 v) {
    float l = len(v);
    if (l < 0.00001f) return {0.0f, 0.0f, 0.0f};
    return {v.x/l, v.y/l, v.z/l};
}

// --- ASYNCHRONOUS STATE TRACKERS ---
// Separating Accel and Mag states fixes the "No Vertical Movement" stutter bug
static Vec3 last_A = {0.0f, 0.0f, 1.0f};
static int64_t t_A = 0;
static Vec3 w_perp = {0.0f, 0.0f, 0.0f};

static Vec3 last_H = {1.0f, 0.0f, 0.0f};
static int64_t t_M = 0;
static Vec3 w_yaw = {0.0f, 0.0f, 0.0f};

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

// 1. Process High-Frequency Pitch/Roll (200Hz)
void process_accel(float ax, float ay, float az, int64_t ts) {
    Vec3 A = normalize({ax, ay, az});
    
    if (t_A != 0) {
        float dt = (ts - t_A) / 1000000000.0f;
        if (dt > 0.001f && dt < 0.1f) {
            // Direct Vector Derivative: cross(A_old, A_new) / dt gives exact angular velocity
            Vec3 delta_rot = cross(last_A, A);
            w_perp.x = delta_rot.x / dt;
            w_perp.y = delta_rot.y / dt;
            w_perp.z = delta_rot.z / dt;
        } else {
            w_perp = {0.0f, 0.0f, 0.0f};
        }
    }
    last_A = A;
    t_A = ts;
}

// 2. Process Low-Frequency Yaw (50Hz)
void process_mag(float mx, float my, float mz, int64_t ts) {
    // Apply factory bias to perfectly center the sphere[span_1](start_span)[span_1](end_span)
    Vec3 M = normalize({mx - HARD_IRON_X, my - HARD_IRON_Y, mz - HARD_IRON_Z});
    
    // Project Magnetic vector flat onto the horizontal plane using Gravity
    float dot_MA = dot(M, last_A);
    Vec3 H = normalize({M.x - last_A.x * dot_MA, M.y - last_A.y * dot_MA, M.z - last_A.z * dot_MA});
    
    if (t_M != 0 && (H.x != 0 || H.y != 0 || H.z != 0)) {
        float dt = (ts - t_M) / 1000000000.0f;
        if (dt > 0.001f && dt < 0.2f) {
            // Measure horizontal spin: cross(H_old, H_new)
            Vec3 delta_spin = cross(last_H, H);
            
            // Project the spin scalar onto the Gravity axis to create pure Yaw
            float yaw_scalar = dot(last_A, delta_spin) / dt;
            w_yaw.x = last_A.x * yaw_scalar;
            w_yaw.y = last_A.y * yaw_scalar;
            w_yaw.z = last_A.z * yaw_scalar;
        } else {
            w_yaw = {0.0f, 0.0f, 0.0f};
        }
    }
    last_H = H;
    t_M = ts;
}

// 3. The Event Interceptor
ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        for (ssize_t i = 0; i < actual_events; i++) {
            
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                process_accel(events[i].acceleration.v[0], events[i].acceleration.v[1], events[i].acceleration.v[2], events[i].timestamp);
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                // Pass raw data, bias is subtracted inside process_mag
                process_mag(events[i].uncalibrated_magnetic.x_uncalib, events[i].uncalibrated_magnetic.y_uncalib, events[i].uncalibrated_magnetic.z_uncalib, events[i].timestamp);
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) {
                process_mag(events[i].magnetic.v[0], events[i].magnetic.v[1], events[i].magnetic.v[2], events[i].timestamp);
            } 
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE) {
                // Combine asynchronous states for continuous, stutter-free output
                float raw_x = w_perp.x + w_yaw.x;
                float raw_y = w_perp.y + w_yaw.y;
                float raw_z = w_perp.z + w_yaw.z;

                events[i].vector.x = (fabs(raw_x) > MAG_NOISE_GATE) ? raw_x * SENSITIVITY : 0.0f;
                events[i].vector.y = (fabs(raw_y) > MAG_NOISE_GATE) ? raw_y * SENSITIVITY : 0.0f;
                events[i].vector.z = (fabs(raw_z) > MAG_NOISE_GATE) ? raw_z * SENSITIVITY : 0.0f;
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
