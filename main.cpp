#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
static const float MAG_NOISE_GATE = 0.005f; 
static const float SENSITIVITY = 1.0f;

// Factory Bias for MT6835 - Permanently centers the mathematical origin[span_15](start_span)[span_15](end_span)
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

struct Vec3 { float x, y, z; };

Vec3 cross_product(Vec3 a, Vec3 b) {
    return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}
float dot_product(Vec3 a, Vec3 b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
Vec3 normalize_vec(Vec3 v) {
    float len = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (len < 0.0001f) return {0.0f, 0.0f, 0.0f};
    return { v.x/len, v.y/len, v.z/len };
}

// Memory States
static float last_accel[3] = {0.0f, 0.0f, 9.81f};
static float last_mag[3] = {0.0f, 1.0f, 0.0f};
static int64_t last_timestamp = 0;

static Vec3 last_A = {0.0f, 1.0f, 0.0f};
static Vec3 last_E = {1.0f, 0.0f, 0.0f};
static Vec3 final_gyro = {0.0f, 0.0f, 0.0f};

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

void compute_sensor_fusion(int64_t timestamp) {
    // A is Gravity (Global World Up Axis)
    Vec3 A = normalize_vec({last_accel[0], last_accel[1], last_accel[2]});
    Vec3 M = normalize_vec({last_mag[0], last_mag[1], last_mag[2]});

    // E is East (Always perfectly horizontal to Gravity)
    Vec3 E = cross_product(M, A);
    float len_E = sqrt(E.x*E.x + E.y*E.y + E.z*E.z);
    if (len_E > 0.001f) {
        E.x /= len_E; E.y /= len_E; E.z /= len_E;
    } else {
        E = last_E; // Singularity guard if perfectly parallel
    }

    if (last_timestamp == 0) {
        last_A = A; last_E = E; last_timestamp = timestamp;
        return;
    }

    float dt = (timestamp - last_timestamp) / 1000000000.0f;
    if (dt <= 0.001f || dt > 0.1f) {
        last_timestamp = timestamp;
        return;
    }

    // ---------------------------------------------------------
    // FPS DECOUPLED KINEMATICS (The 90-Degree Fix)
    // ---------------------------------------------------------

    // 1. LOCAL PITCH (X-Axis) 
    // Extracted by measuring the rotation of Gravity exclusively in the device's Y-Z plane.
    float pitch_sin = last_A.y * A.z - last_A.z * A.y;
    if (pitch_sin > 1.0f) pitch_sin = 1.0f;
    if (pitch_sin < -1.0f) pitch_sin = -1.0f;
    // Negated to match Android's Right-Hand Coordinate Rule
    float raw_pitch = -asin(pitch_sin) / dt;

    // 2. GLOBAL YAW (Y-Axis)
    // Extracted by measuring horizontal turning around the Global World Up axis (Gravity)[span_16](start_span)[span_16](end_span).
    // By dotting the cross-product of East with Gravity, we get pure yaw regardless of pitch angle.
    Vec3 E_cross = cross_product(last_E, E);
    float yaw_sin = dot_product(E_cross, A);
    if (yaw_sin > 1.0f) yaw_sin = 1.0f;
    if (yaw_sin < -1.0f) yaw_sin = -1.0f;
    // Negated to match Android's Right-Hand Coordinate Rule
    float raw_yaw = -asin(yaw_sin) / dt;

    // 3. ISOLATED FPS MAPPING
    // Bypasses the 3D quaternion global/local collapse entirely[span_17](start_span)[span_17](end_span)[span_18](start_span)[span_18](end_span).
    final_gyro.x = (fabs(raw_pitch) > MAG_NOISE_GATE) ? raw_pitch * SENSITIVITY : 0.0f;
    final_gyro.y = (fabs(raw_yaw) > MAG_NOISE_GATE) ? raw_yaw * SENSITIVITY : 0.0f;
    final_gyro.z = 0.0f; // Locked to zero to permanently kill the racing-game tilt

    last_A = A;
    last_E = E;
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
                // Instantly center the vector sphere using exact CPU X hardware bias[span_19](start_span)[span_19](end_span)
                last_mag[0] = events[i].uncalibrated_magnetic.x_uncalib - HARD_IRON_X;
                last_mag[1] = events[i].uncalibrated_magnetic.y_uncalib - HARD_IRON_Y;
                last_mag[2] = events[i].uncalibrated_magnetic.z_uncalib - HARD_IRON_Z;
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
