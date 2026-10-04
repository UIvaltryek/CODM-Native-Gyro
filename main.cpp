#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
static const float SENSITIVITY = 1.0f; 
// EMA Filter Alpha (Replaces hard Noise Gate)[span_12](start_span)[span_12](end_span)
static const float ALPHA = 0.75f; 

// Exact Factory Biases for MT6835
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

struct Vec3 { float x, y, z; };

Vec3 cross(Vec3 a, Vec3 b) {
    return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}
float len(Vec3 v) {
    return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}
Vec3 normalize(Vec3 v) {
    float l = len(v);
    if (l < 0.00001f) return {0.0f, 0.0f, 0.0f};
    return {v.x/l, v.y/l, v.z/l};
}

// --- SYNCHRONOUS STATE TRACKERS ---
// Trackers for the orthogonal basis vectors[span_13](start_span)[span_13](end_span)
static Vec3 last_E = {1.0f, 0.0f, 0.0f};
static Vec3 last_N = {0.0f, 1.0f, 0.0f};
static Vec3 last_A = {0.0f, 0.0f, 1.0f};
static int64_t last_ts = 0;

static float last_ax = 0.0f, last_ay = 0.0f, last_az = 9.81f;
static float last_mx = 0.0f, last_my = 1.0f, last_mz = 0.0f;

// EMA smoothed outputs[span_14](start_span)[span_14](end_span)
static float smooth_x = 0.0f, smooth_y = 0.0f, smooth_z = 0.0f;
static float final_gyro[3] = {0.0f, 0.0f, 0.0f};

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

void compute_sensor_fusion(int64_t ts) {
    // 1. Current Frame Vectors[span_15](start_span)[span_15](end_span)
    Vec3 A = normalize({last_ax, last_ay, last_az});
    Vec3 M = normalize({last_mx - HARD_IRON_X, last_my - HARD_IRON_Y, last_mz - HARD_IRON_Z});

    // Construct Orthonormal Basis (Gravity, East, North)[span_16](start_span)[span_16](end_span)
    Vec3 E = normalize(cross(A, M));
    Vec3 N = cross(E, A);

    if (last_ts == 0) {
        last_A = A; last_E = E; last_N = N;
        last_ts = ts;
        return;
    }

    float dt = (ts - last_ts) / 1000000000.0f;
    if (dt <= 0.0001f || dt > 0.1f) {
        last_ts = ts;
        return;
    }

    // 2. Mathematically extract True 3D Angular Velocity[span_17](start_span)[span_17](end_span)
    // The cross product of the basis vectors across time 
    // gives us the exact rotational velocity for that plane.[span_18](start_span)[span_18](end_span)
    Vec3 w_A = cross(last_A, A);
    Vec3 w_E = cross(last_E, E);
    Vec3 w_N = cross(last_N, N);

    // Sum the rotations and divide by 2*dt[span_19](start_span)[span_19](end_span)
    Vec3 w_total = {
        (w_A.x + w_E.x + w_N.x) / (2.0f * dt),
        (w_A.y + w_E.y + w_N.y) / (2.0f * dt),
        (w_A.z + w_E.z + w_N.z) / (2.0f * dt)
    };

    // 3. EMA Filter (Replaces hard Noise Gate)[span_20](start_span)[span_20](end_span)
    smooth_x = (ALPHA * w_total.x) + ((1.0f - ALPHA) * smooth_x);
    smooth_y = (ALPHA * w_total.y) + ((1.0f - ALPHA) * smooth_y);
    smooth_z = (ALPHA * w_total.z) + ((1.0f - ALPHA) * smooth_z);

    // 4. Output to Gyroscope Events[span_21](start_span)[span_21](end_span)
    // Unlocked Z-axis allows proper view matrix calculation[span_22](start_span)[span_22](end_span)[span_23](start_span)[span_23](end_span)
    final_gyro[0] = smooth_x * SENSITIVITY;
    final_gyro[1] = smooth_y * SENSITIVITY;
    final_gyro[2] = smooth_z * SENSITIVITY;

    // Save state for next frame[span_24](start_span)[span_24](end_span)
    last_A = A; last_E = E; last_N = N; 
    last_ts = ts;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        bool fusion_ready = false;
        int64_t latest_ts = 0;

        for (ssize_t i = 0; i < actual_events; i++) {
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                last_ax = events[i].acceleration.v[0];
                last_ay = events[i].acceleration.v[1];
                last_az = events[i].acceleration.v[2];
                latest_ts = events[i].timestamp;
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                last_mx = events[i].uncalibrated_magnetic.x_uncalib;
                last_my = events[i].uncalibrated_magnetic.y_uncalib;
                last_mz = events[i].uncalibrated_magnetic.z_uncalib;
                if (!fusion_ready) latest_ts = events[i].timestamp; 
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) {
                last_mx = events[i].magnetic.v[0];
                last_my = events[i].magnetic.v[1];
                last_mz = events[i].magnetic.v[2];
                if (!fusion_ready) latest_ts = events[i].timestamp;
                fusion_ready = true;
            } 
            
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE) {
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
