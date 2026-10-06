#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
static const float ALPHA_SMOOTH = 0.65f; // Cures the raw derivative wobble
static const float MAG_NOISE_GATE = 0.015f; 

// --- FACTORY BIASES (MT6835 Hard-Iron Offsets) ---
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

static float smoothed_gyro[3] = {0.0f, 0.0f, 0.0f}; 

// Sensor States
static float last_A[3] = {0.0f, 0.0f, 1.0f}; 
static float last_M[3] = {0.0f, 1.0f, 0.0f}; 
static float last_H[3] = {0.0f, 1.0f, 0.0f}; 
static int64_t last_timestamp = 0; 

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t); 
static getEvents_t orig_getEvents = nullptr; 
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

void normalize(float v[3]) {
    float norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (norm > 0.0001f) { v[0] /= norm; v[1] /= norm; v[2] /= norm; }
}

void cross_product(float a[3], float b[3], float out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

float dot_product(float a[3], float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void compute_vector_kinematics(int64_t timestamp) {
    if (last_timestamp == 0) { 
        last_timestamp = timestamp; 
        return; 
    }

    float raw_dt = (timestamp - last_timestamp) / 1000000000.0f; 
    if (raw_dt <= 0.0f || raw_dt > 0.1f) { 
        last_timestamp = timestamp; 
        return; 
    }

    // 1. Current Gravity (A) and Magnetic (M) Vectors
    float A[3] = {last_A[0], last_A[1], last_A[2]};
    float M[3] = {last_M[0], last_M[1], last_M[2]};

    // 2. Extract the true horizontal Magnetic vector (H), perfectly perpendicular to Gravity
    float M_dot_A = dot_product(M, A);
    float H[3] = {
        M[0] - M_dot_A * A[0],
        M[1] - M_dot_A * A[1],
        M[2] - M_dot_A * A[2]
    };
    
    // Prevent division by zero if vectors perfectly align
    float H_norm = sqrt(H[0]*H[0] + H[1]*H[1] + H[2]*H[2]);
    if (H_norm > 0.001f) {
        H[0] /= H_norm; H[1] /= H_norm; H[2] /= H_norm;
    } else {
        H[0] = last_H[0]; H[1] = last_H[1]; H[2] = last_H[2];
    }

    // ====================================================================
    // PART 1: TILT VELOCITY (Pitch/Roll)
    // Formula: dA x A
    // Flawlessly calculates vertical aiming without using atan2 (ZERO 180 SNAPS)
    // ====================================================================
    float dA[3] = {
        (A[0] - last_A[0]) / raw_dt,
        (A[1] - last_A[1]) / raw_dt,
        (A[2] - last_A[2]) / raw_dt
    };
    float w_tilt[3];
    cross_product(dA, A, w_tilt);

    // ====================================================================
    // PART 2: YAW VELOCITY (Horizontal Panning)
    // Measures how fast H rotates strictly around the A axis
    // ====================================================================
    float C[3];
    cross_product(last_H, H, C);
    
    // The dot product with Gravity gives the exact radians swept.
    // Negative sign correctly aligns the rotation with right-hand rule kinematics.
    float yaw_delta = -dot_product(C, A); 
    float w_yaw_scalar = yaw_delta / raw_dt;
    
    if (fabs(w_yaw_scalar) < MAG_NOISE_GATE) w_yaw_scalar = 0.0f;
    
    // Project the horizontal speed perfectly onto the Gravity axis
    float w_yaw[3] = {
        w_yaw_scalar * A[0],
        w_yaw_scalar * A[1],
        w_yaw_scalar * A[2]
    };

    // ====================================================================
    // PART 3: MASTER SYNTHESIS & ANTI-WOBBLE FILTER
    // ====================================================================
    float raw_x = w_tilt[0] + w_yaw[0];
    float raw_y = w_tilt[1] + w_yaw[1];
    float raw_z = w_tilt[2] + w_yaw[2];

    // Your 0.65f filter to surgically kill the derivative wobble
    smoothed_gyro[0] = (ALPHA_SMOOTH * raw_x) + ((1.0f - ALPHA_SMOOTH) * smoothed_gyro[0]);
    smoothed_gyro[1] = (ALPHA_SMOOTH * raw_y) + ((1.0f - ALPHA_SMOOTH) * smoothed_gyro[1]);
    smoothed_gyro[2] = (ALPHA_SMOOTH * raw_z) + ((1.0f - ALPHA_SMOOTH) * smoothed_gyro[2]);

    // Update States
    last_H[0] = H[0]; last_H[1] = H[1]; last_H[2] = H[2];
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
                last_A[0] = events[i].acceleration.x;
                last_A[1] = events[i].acceleration.y;
                last_A[2] = events[i].acceleration.z;
                normalize(last_A);
                fusion_ready = true;
            } 
            // RESTORED UNCALIBRATED MAG WITH YOUR HARDCODED BIASES
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                last_M[0] = events[i].uncalibrated_magnetic.x_uncalib - HARD_IRON_X;
                last_M[1] = events[i].uncalibrated_magnetic.y_uncalib - HARD_IRON_Y;
                last_M[2] = events[i].uncalibrated_magnetic.z_uncalib - HARD_IRON_Z;
                normalize(last_M);
                fusion_ready = true;
            }
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE || events[i].type == 4 || events[i].type == 16) {
                events[i].vector.x = smoothed_gyro[0]; 
                events[i].vector.y = smoothed_gyro[1]; 
                events[i].vector.z = smoothed_gyro[2]; 
            }
        }

        if (fusion_ready && latest_ts > 0) {
            compute_vector_kinematics(latest_ts);
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
