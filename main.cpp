#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// Production Build: Quaternion Velocity Extraction + 9006.png Custom Mapping
static const float ALPHA_ACCEL = 0.65f; 
static const float MAG_NOISE_GATE = 0.015f; 

// --- FACTORY BIASES (MT6835 Hard-Iron Offsets) ---
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

static float smoothed_gyro[3] = {0.0f, 0.0f, 0.0f}; 

// Sensor States
static float last_accel[3] = {0.0f, 0.0f, 9.81f}; 
static float last_mag[3] = {0.0f, 1.0f, 0.0f}; 
static int64_t last_timestamp = 0; 
static float last_q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; 

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

void compute_quaternion_kinematics(int64_t timestamp) {
    float A[3] = {last_accel[0], last_accel[1], last_accel[2]};
    normalize(A); 
    
    float M[3] = {last_mag[0], last_mag[1], last_mag[2]};
    normalize(M);

    // 1. Build Orthogonal Basis safely
    float E[3]; cross_product(M, A, E); 
    float E_norm = sqrt(E[0]*E[0] + E[1]*E[1] + E[2]*E[2]);
    if (E_norm < 0.001f) { 
        E[0] = 1.0f; E[1] = 0.0f; E[2] = 0.0f; 
    } else { 
        E[0]/=E_norm; E[1]/=E_norm; E[2]/=E_norm; 
    }
    
    float N[3]; cross_product(A, E, N); 
    normalize(N); 

    float R[3][3] = {
        {E[0], N[0], A[0]},
        {E[1], N[1], A[1]},
        {E[2], N[2], A[2]}
    };

    float q_curr[4];
    matrix_to_quat(R, q_curr);

    if (last_timestamp == 0) { 
        last_timestamp = timestamp; 
        memcpy(last_q, q_curr, sizeof(q_curr));
        return; 
    }

    float raw_dt = (timestamp - last_timestamp) / 1000000000.0f; 
    if (raw_dt <= 0.0f || raw_dt > 0.1f) { 
        last_timestamp = timestamp; 
        return; 
    }

    // 2. Quaternion Time Derivative (Immune to atan2 Gimbal Lock snaps)
    float dq_w = last_q[0]*q_curr[0] + last_q[1]*q_curr[1] + last_q[2]*q_curr[2] + last_q[3]*q_curr[3];
    float dq_x = last_q[0]*q_curr[1] - last_q[1]*q_curr[0] - last_q[2]*q_curr[3] + last_q[3]*q_curr[2];
    float dq_y = last_q[0]*q_curr[2] + last_q[1]*q_curr[3] - last_q[2]*q_curr[0] - last_q[3]*q_curr[1];
    float dq_z = last_q[0]*q_curr[3] - last_q[1]*q_curr[2] + last_q[2]*q_curr[1] - last_q[3]*q_curr[0];

    if (dq_w < 0.0f) {
        dq_w = -dq_w; dq_x = -dq_x; dq_y = -dq_y; dq_z = -dq_z;
    }

    float wx = (2.0f * dq_x) / raw_dt;
    float wy = (2.0f * dq_y) / raw_dt;
    float wz = (2.0f * dq_z) / raw_dt;

    // 3. Dynamically route the 3D velocities into your exact 2D screen mapping
    float speed_pitch = (wx * A[1]) - (wy * A[0]);               // Vertical Aim
    float speed_roll  = (wy * A[1]) + (wx * A[0]);               // Steering Wheel
    float speed_yaw   = (wx * A[0]) + (wy * A[1]) + (wz * A[2]); // Bicycle Handlebars

    if (fabs(speed_yaw) < MAG_NOISE_GATE) speed_yaw = 0.0f; 

    // 4. Your Universal Fade Factor (Uses Z-Gravity to detect TV hold vs flat)
    float fade_factor = pow(1.0f - (A[2] * A[2]), 2.0f); 

    // 5. Your precise 9006.png horizontal synthesis
    float final_horizontal_speed = (speed_roll * (1.0f - fade_factor)) + (speed_yaw * fade_factor); 

    // 6. Direct output mapping with Z-axis cull
    smoothed_gyro[0] = ALPHA_ACCEL * speed_pitch + (1.0f - ALPHA_ACCEL) * smoothed_gyro[0]; 
    smoothed_gyro[1] = ALPHA_ACCEL * final_horizontal_speed + (1.0f - ALPHA_ACCEL) * smoothed_gyro[1]; 
    smoothed_gyro[2] = 0.0f; 

    memcpy(last_q, q_curr, sizeof(q_curr));
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
                last_accel[0] = events[i].acceleration.x;
                last_accel[1] = events[i].acceleration.y;
                last_accel[2] = events[i].acceleration.z;
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                last_mag[0] = events[i].uncalibrated_magnetic.x_uncalib - HARD_IRON_X;
                last_mag[1] = events[i].uncalibrated_magnetic.y_uncalib - HARD_IRON_Y;
                last_mag[2] = events[i].uncalibrated_magnetic.z_uncalib - HARD_IRON_Z;
                fusion_ready = true;
            }
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE || events[i].type == 4 || events[i].type == 16) {
                events[i].vector.x = smoothed_gyro[0]; 
                events[i].vector.y = smoothed_gyro[1]; 
                events[i].vector.z = smoothed_gyro[2]; 
            }
        }

        if (fusion_ready && latest_ts > 0) {
            compute_quaternion_kinematics(latest_ts);
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
