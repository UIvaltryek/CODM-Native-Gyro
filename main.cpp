#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
static const float SENSITIVITY = 1.0f;
static const float LOW_PASS_ALPHA = 0.4f; // Step 4: Filter the Result (Lower = smoother, Higher = more responsive)
static const float STATIC_NOISE_GATE = 0.002f;

// --- FACTORY BIASES (MT6835 Hard-Iron Offsets) ---
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

static float current_A[3] = {0.0f, 0.0f, 1.0f};
static float current_M[3] = {0.0f, 1.0f, 0.0f};

// History state for Step 2 and Step 4
static int64_t last_timestamp = 0;
static float last_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float filtered_gyro[3] = {0.0f, 0.0f, 0.0f};

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

void matrix_to_quat(float R[3][3], float q[4]) {
    float tr = R[0][0] + R[1][1] + R[2][2];
    if (tr > 0.0f) { 
        float S = sqrt(tr + 1.0f) * 2.0f; 
        q[0] = 0.25f * S; 
        q[1] = (R[1][2] - R[2][1]) / S; 
        q[2] = (R[2][0] - R[0][2]) / S; 
        q[3] = (R[0][1] - R[1][0]) / S; 
    } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) { 
        float S = sqrt(1.0f + R[0][0] - R[1][1] - R[2][2]) * 2.0f; 
        q[0] = (R[1][2] - R[2][1]) / S; 
        q[1] = 0.25f * S; 
        q[2] = (R[0][1] + R[1][0]) / S; 
        q[3] = (R[2][0] + R[0][2]) / S; 
    } else if (R[1][1] > R[2][2]) { 
        float S = sqrt(1.0f + R[1][1] - R[0][0] - R[2][2]) * 2.0f; 
        q[0] = (R[2][0] - R[0][2]) / S; 
        q[1] = (R[0][1] + R[1][0]) / S; 
        q[2] = 0.25f * S; 
        q[3] = (R[1][2] + R[2][1]) / S; 
    } else { 
        float S = sqrt(1.0f + R[2][2] - R[0][0] - R[1][1]) * 2.0f; 
        q[0] = (R[0][1] - R[1][0]) / S; 
        q[1] = (R[2][0] + R[0][2]) / S; 
        q[2] = (R[1][2] + R[2][1]) / S; 
        q[3] = 0.25f * S; 
    }
    float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 0.0001f) { q[0]/=norm; q[1]/=norm; q[2]/=norm; q[3]/=norm; }
}

void compute_nxp_kinematics(int64_t current_timestamp) {
    if (last_timestamp == 0) {
        last_timestamp = current_timestamp;
        return;
    }

    // Step 3: Divide by Time (Calculate delta t)
    float dt = (current_timestamp - last_timestamp) / 1000000000.0f;
    last_timestamp = current_timestamp;
    if (dt <= 0.0f || dt > 0.1f) return; // Ignore frozen or massive time leaps

    // Step 1: Compute Orientation q(t)
    float H[3]; cross_product(current_M, current_A, H); normalize(H);
    float N[3]; cross_product(current_A, H, N); normalize(N);
    
    float R[3][3] = {
        {H[0], H[1], H[2]},
        {N[0], N[1], N[2]},
        {current_A[0], current_A[1], current_A[2]}
    };

    float q_curr[4]; 
    matrix_to_quat(R, q_curr);

    // Step 2: Calculate Difference (Relative rotation: q_diff = q_curr * q_prev_inverse)
    float q_prev_inv[4] = {last_q[0], -last_q[1], -last_q[2], -last_q[3]};
    float r[4];
    r[0] = q_curr[0]*q_prev_inv[0] - q_curr[1]*q_prev_inv[1] - q_curr[2]*q_prev_inv[2] - q_curr[3]*q_prev_inv[3];
    r[1] = q_curr[0]*q_prev_inv[1] + q_curr[1]*q_prev_inv[0] + q_curr[2]*q_prev_inv[3] - q_curr[3]*q_prev_inv[2];
    r[2] = q_curr[0]*q_prev_inv[2] - q_curr[1]*q_prev_inv[3] + q_curr[2]*q_prev_inv[0] + q_curr[3]*q_prev_inv[1];
    r[3] = q_curr[0]*q_prev_inv[3] + q_curr[1]*q_prev_inv[2] - q_curr[2]*q_prev_inv[1] + q_curr[3]*q_prev_inv[0];

    // Enforce shortest path rotation to prevent mathematical flipping
    if (r[0] < 0.0f) {
        r[0] = -r[0]; r[1] = -r[1]; r[2] = -r[2]; r[3] = -r[3];
    }
    memcpy(last_q, q_curr, sizeof(q_curr));

    // Extract the raw angular velocity (Radians per second)
    float angle = 2.0f * acos(fmin(1.0f, r[0]));
    float s = sqrt(1.0f - r[0]*r[0]);
    
    float raw_x = 0.0f, raw_y = 0.0f, raw_z = 0.0f;
    if (s > 0.001f) {
        raw_x = (r[1] / s) * (angle / dt) * SENSITIVITY;
        raw_y = (r[2] / s) * (angle / dt) * SENSITIVITY;
        raw_z = (r[3] / s) * (angle / dt) * SENSITIVITY;
    }

    // Step 4: Filter the Result (Low-Pass Filter to minimize numerical differentiation noise)
    filtered_gyro[0] = (LOW_PASS_ALPHA * raw_x) + ((1.0f - LOW_PASS_ALPHA) * filtered_gyro[0]);
    filtered_gyro[1] = (LOW_PASS_ALPHA * raw_y) + ((1.0f - LOW_PASS_ALPHA) * filtered_gyro[1]);
    filtered_gyro[2] = (LOW_PASS_ALPHA * raw_z) + ((1.0f - LOW_PASS_ALPHA) * filtered_gyro[2]);

    // Apply strict hardware noise gate
    if (fabs(filtered_gyro[0]) < STATIC_NOISE_GATE) filtered_gyro[0] = 0.0f;
    if (fabs(filtered_gyro[1]) < STATIC_NOISE_GATE) filtered_gyro[1] = 0.0f;
    if (fabs(filtered_gyro[2]) < STATIC_NOISE_GATE) filtered_gyro[2] = 0.0f;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        bool fusion_ready = false;
        int64_t latest_ts = 0;

        for (ssize_t i = 0; i < actual_events; i++) {
            latest_ts = events[i].timestamp;
            
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                current_A[0] = events[i].acceleration.x;
                current_A[1] = events[i].acceleration.y;
                current_A[2] = events[i].acceleration.z;
                normalize(current_A);
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                current_M[0] = events[i].uncalibrated_magnetic.x_uncalib - HARD_IRON_X;
                current_M[1] = events[i].uncalibrated_magnetic.y_uncalib - HARD_IRON_Y;
                current_M[2] = events[i].uncalibrated_magnetic.z_uncalib - HARD_IRON_Z;
                normalize(current_M);
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE || events[i].type == 4 || events[i].type == 16) {
                events[i].vector.x = filtered_gyro[0]; 
                events[i].vector.y = filtered_gyro[1]; 
                events[i].vector.z = filtered_gyro[2]; 
            }
        }

        if (fusion_ready && latest_ts > 0) {
            compute_nxp_kinematics(latest_ts);
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
