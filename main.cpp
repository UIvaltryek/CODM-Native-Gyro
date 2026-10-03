#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
// Final Output Smoothing (0.1 to 0.9). Higher = smoother but more delay.
static const float ALPHA_OUT = 0.65f; 
static const float MAG_NOISE_GATE = 0.005f; 
static const float SENSITIVITY = 1.0f;

// --- 3D / 4D MATH STRUCTURES ---
struct Vec3 { float x, y, z; };
struct Quat { float w, x, y, z; };

// Vector Math
Vec3 cross_product(Vec3 a, Vec3 b) {
    return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}
Vec3 normalize_vec(Vec3 v) {
    float len = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (len < 0.0001f) return {0.0f, 0.0f, 0.0f};
    return { v.x/len, v.y/len, v.z/len };
}

// Quaternion Math
Quat q_conjugate(Quat q) {
    return { q.w, -q.x, -q.y, -q.z };
}
Quat q_multiply(Quat q1, Quat q2) {
    return {
        q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
        q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
        q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
        q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
    };
}
Quat normalize_quat(Quat q) {
    float len = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (len < 0.0001f) return {1.0f, 0.0f, 0.0f, 0.0f};
    return { q.w/len, q.x/len, q.y/len, q.z/len };
}

// Memory States
static float last_accel[3] = {0.0f, 0.0f, 9.81f};
static float last_mag[3] = {0.0f, 1.0f, 0.0f};
static int64_t last_timestamp = 0;

static Quat last_q = {1.0f, 0.0f, 0.0f, 0.0f};
static Vec3 smoothed_gyro = {0.0f, 0.0f, 0.0f};

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;

typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

// Robust, Singularity-Free Matrix to Quaternion Conversion
Quat matrix_to_quat(Vec3 E, Vec3 N, Vec3 A) {
    Quat q;
    float trace = E.x + N.y + A.z;
    
    if (trace > 0.0f) {
        float s = sqrt(trace + 1.0f) * 2.0f;
        q.w = 0.25f * s;
        q.x = (N.z - A.y) / s;
        q.y = (A.x - E.z) / s;
        q.z = (E.y - N.x) / s;
    } else if ((E.x > N.y) && (E.x > A.z)) {
        float s = sqrt(1.0f + E.x - N.y - A.z) * 2.0f;
        q.w = (N.z - A.y) / s;
        q.x = 0.25f * s;
        q.y = (N.x + E.y) / s;
        q.z = (A.x + E.z) / s;
    } else if (N.y > A.z) {
        float s = sqrt(1.0f + N.y - E.x - A.z) * 2.0f;
        q.w = (A.x - E.z) / s;
        q.x = (N.x + E.y) / s;
        q.y = 0.25f * s;
        q.z = (A.y + N.z) / s;
    } else {
        float s = sqrt(1.0f + A.z - E.x - N.y) * 2.0f;
        q.w = (E.y - N.x) / s;
        q.x = (A.x + E.z) / s;
        q.y = (A.y + N.z) / s;
        q.z = 0.25f * s;
    }
    return normalize_quat(q);
}

void compute_sensor_fusion(int64_t timestamp) {
    // 1. Raw Vectors
    Vec3 A = {last_accel[0], last_accel[1], last_accel[2]};
    Vec3 M = {last_mag[0], last_mag[1], last_mag[2]};

    // 2. Normalize Gravity (A)
    A = normalize_vec(A);

    // 3. Magnetic East (E) = M x A
    Vec3 E = cross_product(M, A);
    E = normalize_vec(E);

    // 4. Magnetic North (N) = A x E
    Vec3 N = cross_product(A, E);
    N = normalize_vec(N); // Orthogonal basis is now complete

    // 5. Build Current Quaternion from Orthogonal Matrix
    Quat current_q = matrix_to_quat(E, N, A);

    // Ensure we take the shortest rotational path
    float dot = current_q.w*last_q.w + current_q.x*last_q.x + current_q.y*last_q.y + current_q.z*last_q.z;
    if (dot < 0.0f) {
        current_q.w = -current_q.w; current_q.x = -current_q.x; 
        current_q.y = -current_q.y; current_q.z = -current_q.z;
    }

    if (last_timestamp == 0) {
        last_timestamp = timestamp;
        last_q = current_q;
        return;
    }

    float raw_dt = (timestamp - last_timestamp) / 1000000000.0f; 
    if (raw_dt <= 0.001f || raw_dt > 0.1f) {
        last_timestamp = timestamp;
        return;
    }

    // 6. Calculate Rotational Delta (Current * Conjugate(Previous))
    Quat q_delta = q_multiply(current_q, q_conjugate(last_q));

    // 7. Convert 4D Delta directly to 3D Angular Velocity (rad/s)
    float raw_wx = (2.0f * q_delta.x) / raw_dt;
    float raw_wy = (2.0f * q_delta.y) / raw_dt;
    float raw_wz = (2.0f * q_delta.z) / raw_dt;

    // Apply Noise Gate
    float target_x = (fabs(raw_wx) > MAG_NOISE_GATE) ? raw_wx * SENSITIVITY : 0.0f;
    float target_y = (fabs(raw_wy) > MAG_NOISE_GATE) ? raw_wy * SENSITIVITY : 0.0f;
    float target_z = (fabs(raw_wz) > MAG_NOISE_GATE) ? raw_wz * SENSITIVITY : 0.0f;

    // 8. Exponential Moving Average for smooth sniper tracking
    smoothed_gyro.x = ALPHA_OUT * target_x + (1.0f - ALPHA_OUT) * smoothed_gyro.x;
    smoothed_gyro.y = ALPHA_OUT * target_y + (1.0f - ALPHA_OUT) * smoothed_gyro.y;
    smoothed_gyro.z = ALPHA_OUT * target_z + (1.0f - ALPHA_OUT) * smoothed_gyro.z;

    // Save state for next frame
    last_q = current_q;
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
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                last_mag[0] = events[i].uncalibrated_magnetic.x_uncalib;
                last_mag[1] = events[i].uncalibrated_magnetic.y_uncalib;
                last_mag[2] = events[i].uncalibrated_magnetic.z_uncalib;
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
                events[i].vector.x = smoothed_gyro.x; 
                events[i].vector.y = smoothed_gyro.y; 
                events[i].vector.z = smoothed_gyro.z; 
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
