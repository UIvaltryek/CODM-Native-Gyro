#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
static const float SENSITIVITY = 1.0f; 
static const float STATIC_NOISE_GATE = 0.002f; 

// --- QUATERNION SLIDING WINDOW ---
class RegressionBuffer {
public:
    int64_t t[5] = {0};
    float w[5] = {0}, x[5] = {0}, y[5] = {0}, z[5] = {0};
    int index = 0, count = 0;

    void push(int64_t time_nanos, float qw, float qx, float qy, float qz) {
        t[index] = time_nanos; w[index] = qw; x[index] = qx; y[index] = qy; z[index] = qz;
        index = (index + 1) % 5;
        if (count < 5) count++;
    }

    void derive(float& dw, float& dx, float& dy, float& dz) {
        if (count < 2) { dw = dx = dy = dz = 0.0f; return; }
        
        double sum_t = 0, sum_w = 0, sum_x = 0, sum_y = 0, sum_z = 0;
        int64_t t0 = t[0]; 
        for (int i = 0; i < count; i++) {
            double dt = (t[i] - t0) / 1000000000.0;
            sum_t += dt; sum_w += w[i]; sum_x += x[i]; sum_y += y[i]; sum_z += z[i];
        }
        double mean_t = sum_t / count;
        double mean_w = sum_w / count;
        double mean_x = sum_x / count;
        double mean_y = sum_y / count;
        double mean_z = sum_z / count;

        double num_w = 0, num_x = 0, num_y = 0, num_z = 0, den = 0;
        for (int i = 0; i < count; i++) {
            double dt = ((t[i] - t0) / 1000000000.0) - mean_t;
            den += dt * dt;
            num_w += dt * (w[i] - mean_w);
            num_x += dt * (x[i] - mean_x);
            num_y += dt * (y[i] - mean_y);
            num_z += dt * (z[i] - mean_z);
        }
        
        if (den < 1e-9) { dw = dx = dy = dz = 0.0f; return; } 
        
        dw = (float)(num_w / den); dx = (float)(num_x / den); 
        dy = (float)(num_y / den); dz = (float)(num_z / den);
    }
};

static RegressionBuffer quat_buffer;
static float current_A[3] = {0.0f, 0.0f, 1.0f};
static float current_M[3] = {0.0f, 1.0f, 0.0f};
static float last_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float final_gyro[3] = {0.0f, 0.0f, 0.0f};

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
    // 1. Build the rigid 3D Grid (Rotation Matrix)
    float H[3]; // East
    cross_product(current_M, current_A, H);
    normalize(H);
    
    float N[3]; // North
    cross_product(current_A, H, N);
    normalize(N);
    
    float R[3][3] = {
        {H[0], H[1], H[2]},
        {N[0], N[1], N[2]},
        {current_A[0], current_A[1], current_A[2]}
    };

    // 2. Convert to Quaternion (Immune to Gimbal Lock at 90 deg)
    float q[4]; // w, x, y, z
    matrix_to_quat(R, q);

    // 3. Double-Cover safety check (Prevents random math flips from causing gyro spikes)
    if ((q[0]*last_q[0] + q[1]*last_q[1] + q[2]*last_q[2] + q[3]*last_q[3]) < 0.0f) {
        q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3];
    }
    memcpy(last_q, q, sizeof(q));

    // 4. Push to 25ms regression buffer and extract the smooth slope
    quat_buffer.push(timestamp, q[0], q[1], q[2], q[3]);
    float dw, dx, dy, dz;
    quat_buffer.derive(dw, dx, dy, dz);

    // 5. Aerospace extraction: w = 2 * (q_dot * q_inverse)
    // This perfectly extracts the 3D angular velocity from the solid 3D grid!
    float raw_x = 2.0f * (-dw * q[1] + dx * q[0] - dy * q[3] + dz * q[2]);
    float raw_y = 2.0f * (-dw * q[2] + dx * q[3] + dy * q[0] - dz * q[1]);
    float raw_z = 2.0f * (-dw * q[3] - dx * q[2] + dy * q[1] + dz * q[0]);

    raw_x *= SENSITIVITY;
    raw_y *= SENSITIVITY;
    raw_z *= SENSITIVITY;

    final_gyro[0] = (fabs(raw_x) > STATIC_NOISE_GATE) ? raw_x : 0.0f;
    final_gyro[1] = (fabs(raw_y) > STATIC_NOISE_GATE) ? raw_y : 0.0f;
    final_gyro[2] = (fabs(raw_z) > STATIC_NOISE_GATE) ? raw_z : 0.0f;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        bool fusion_ready = false;
        int64_t last_ts = 0;

        for (ssize_t i = 0; i < actual_events; i++) {
            last_ts = events[i].timestamp;
            
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                current_A[0] = events[i].acceleration.x;
                current_A[1] = events[i].acceleration.y;
                current_A[2] = events[i].acceleration.z;
                normalize(current_A);
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                current_M[0] = events[i].uncalibrated_magnetic.x_uncalib;
                current_M[1] = events[i].uncalibrated_magnetic.y_uncalib;
                current_M[2] = events[i].uncalibrated_magnetic.z_uncalib;
                normalize(current_M);
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD || events[i].type == 2) {
                current_M[0] = events[i].magnetic.x;
                current_M[1] = events[i].magnetic.y;
                current_M[2] = events[i].magnetic.z;
                normalize(current_M);
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE || events[i].type == 4 || events[i].type == 16) {
                events[i].vector.x = final_gyro[0]; 
                events[i].vector.y = final_gyro[1]; 
                events[i].vector.z = final_gyro[2]; 
            }
        }

        if (fusion_ready) {
            compute_quaternion_kinematics(last_ts);
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
