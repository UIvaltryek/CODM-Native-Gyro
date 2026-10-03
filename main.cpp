#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
static const float MAG_NOISE_GATE = 0.005f; 
static const float SENSITIVITY = 1.0f;

struct Vec3 { float x, y, z; };
struct Quat { float w, x, y, z; };

Vec3 cross_product(Vec3 a, Vec3 b) {
    return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}
Vec3 normalize_vec(Vec3 v) {
    float len = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (len < 0.0001f) return {0.0f, 0.0f, 0.0f};
    return { v.x/len, v.y/len, v.z/len };
}

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
    float len = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    return { q.w/len, q.x/len, q.y/len, q.z/len };
}

static float last_accel[3] = {0.0f, 0.0f, 9.81f};
static float last_mag[3] = {0.0f, 1.0f, 0.0f};
static int64_t last_timestamp = 0;

static Quat last_q = {1.0f, 0.0f, 0.0f, 0.0f};
static Vec3 final_gyro = {0.0f, 0.0f, 0.0f};

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

void compute_sensor_fusion(int64_t timestamp) {
    Vec3 A = normalize_vec({last_accel[0], last_accel[1], last_accel[2]});
    Vec3 M = normalize_vec({last_mag[0], last_mag[1], last_mag[2]});

    Vec3 E = normalize_vec(cross_product(M, A));
    Vec3 N = normalize_vec(cross_product(A, E));

    Quat current_q = matrix_to_quat(E, N, A);

    if (last_timestamp == 0) {
        last_timestamp = timestamp;
        last_q = current_q;
        return;
    }

    float dt = (timestamp - last_timestamp) / 1000000000.0f;
    if (dt <= 0.001f || dt > 0.1f) {
        last_timestamp = timestamp;
        return;
    }

    // Shortest path check
    float dot = current_q.w*last_q.w + current_q.x*last_q.x + current_q.y*last_q.y + current_q.z*last_q.z;
    if (dot < 0.0f) {
        current_q.w = -current_q.w; current_q.x = -current_q.x; 
        current_q.y = -current_q.y; current_q.z = -current_q.z;
    }

    // Quaternion Conjugate for previous frame inverse
    Quat inv_last = { last_q.w, -last_q.x, -last_q.y, -last_q.z };

    // Delta Q = Current * Inverse(Previous)
    Quat dq = {
        current_q.w*inv_last.w - current_q.x*inv_last.x - current_q.y*inv_last.y - current_q.z*inv_last.z,
        current_q.w*inv_last.x + current_q.x*inv_last.w + current_q.y*inv_last.z - current_q.z*inv_last.y,
        current_q.w*inv_last.y - current_q.x*inv_last.z + current_q.y*inv_last.w + current_q.z*inv_last.x,
        current_q.w*inv_last.z + current_q.x*inv_last.y - current_q.y*inv_last.x + current_q.z*inv_last.w
    };

    // Extract raw angular velocity directly from quaternion vector components
    float raw_x = (2.0f * dq.x) / dt;
    float raw_y = (2.0f * dq.y) / dt;
    float raw_z = (2.0f * dq.z) / dt;

    final_gyro.x = (fabs(raw_x) > MAG_NOISE_GATE) ? raw_x * SENSITIVITY : 0.0f;
    final_gyro.y = (fabs(raw_y) > MAG_NOISE_GATE) ? raw_y * SENSITIVITY : 0.0f;
    final_gyro.z = (fabs(raw_z) > MAG_NOISE_GATE) ? raw_z * SENSITIVITY : 0.0f;

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
                events[i].vector.x = final_gyro.x; 
                events[i].vector.y = final_gyro.y; 
                events[i].vector.z = final_gyro.z; 
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
