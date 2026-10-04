#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
static const float MAHONY_KP = 15.0f; 
static const float SENSITIVITY = 1.0f;

// Factory Bias for MT6835 - Permanently centers the mathematical origin
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

// Display Transformation (0 = Portrait, 1 = Landscape Left, 2 = Landscape Right)
// CoD Mobile is usually played in Landscape Left (Rotation 90)
static const int DISPLAY_ORIENTATION = 1; 

struct Quat { float w, x, y, z; };

inline float inv_sqrt(float x) {
    return 1.0f / sqrtf(x);
}

// --- UNIFIED CONTINUOUS STATE ---
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static Quat last_q = {1.0f, 0.0f, 0.0f, 0.0f};

// Synchronized Sensor Caches
static float last_ax = 0.0f, last_ay = 0.0f, last_az = 9.81f;
static float last_mx = 0.0f, last_my = 1.0f, last_mz = 0.0f;
static int64_t unified_ts = 0;
static float final_gyro[3] = {0.0f, 0.0f, 0.0f};

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

// 1. Continuous 3D Orientation Estimator
void update_orientation(float dt) {
    float ax = last_ax, ay = last_ay, az = last_az;
    float mx = last_mx, my = last_my, mz = last_mz;

    // Normalize
    float norm_a = inv_sqrt(ax * ax + ay * ay + az * az);
    if (isnan(norm_a)) return;
    ax *= norm_a; ay *= norm_a; az *= norm_a;

    // 4. Reject/hold magnetometer updates when degenerate (Looking straight up/down)
    // If the Z-axis of gravity is > 0.98 (approx 80-90 degrees pitch), we temporarily 
    // drop the magnetic weight to prevent coordinate degeneracy.
    float mag_weight = (fabsf(az) > 0.98f) ? 0.1f : 1.0f; 

    float norm_m = inv_sqrt(mx * mx + my * my + mz * mz);
    if (isnan(norm_m)) return;
    mx *= norm_m; my *= norm_m; mz *= norm_m;

    float q0q0 = q0*q0, q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3;
    float q1q1 = q1*q1, q1q2 = q1*q2, q1q3 = q1*q3;
    float q2q2 = q2*q2, q2q3 = q2*q3, q3q3 = q3*q3;

    float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    float hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    float bx = sqrtf(hx * hx + hy * hy);
    float bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    float halfvx = q1q3 - q0q2;
    float halfvy = q0q1 + q2q3;
    float halfvz = q0q0 - 0.5f + q3q3;

    float halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    float halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    float halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // Cross-product error 
    float halfex = (ay * halfvz - az * halfvy) + ((my * halfwz - mz * halfwy) * mag_weight);
    float halfey = (az * halfvx - ax * halfvz) + ((mz * halfwx - mx * halfwz) * mag_weight);
    float halfez = (ax * halfvy - ay * halfvx) + ((mx * halfwy - my * halfwx) * mag_weight);

    float wx = 2.0f * MAHONY_KP * halfex;
    float wy = 2.0f * MAHONY_KP * halfey;
    float wz = 2.0f * MAHONY_KP * halfez;

    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * wx - qc * wy - q3 * wz) * (0.5f * dt);
    q1 += ( qa * wx + qc * wz - q3 * wy) * (0.5f * dt);
    q2 += ( qa * wy - qb * wz + q3 * wx) * (0.5f * dt);
    q3 += ( qa * wz + qb * wy - qc * wx) * (0.5f * dt);

    float norm_q = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= norm_q; q1 *= norm_q; q2 *= norm_q; q3 *= norm_q;
}

// 2. Differentiate Orientation -> Angular Velocity
void derive_angular_velocity(float dt) {
    Quat current_q = {q0, q1, q2, q3};
    Quat inv_last = {last_q.w, -last_q.x, -last_q.y, -last_q.z};

    // Local Frame Delta
    Quat dq = {
        inv_last.w*current_q.w - inv_last.x*current_q.x - inv_last.y*current_q.y - inv_last.z*current_q.z,
        inv_last.w*current_q.x + inv_last.x*current_q.w + inv_last.y*current_q.z - inv_last.z*current_q.y,
        inv_last.w*current_q.y - inv_last.x*current_q.z + inv_last.y*current_q.w + inv_last.z*current_q.x,
        inv_last.w*current_q.z + inv_last.x*current_q.y - inv_last.y*current_q.x + inv_last.z*current_q.w
    };

    // 2 * dq / dt
    float raw_wx = -(2.0f * dq.x) / dt;
    float raw_wy = -(2.0f * dq.y) / dt;
    float raw_wz = -(2.0f * dq.z) / dt;

    // 2. Explicit Android Display-Axis Transformation
    float out_x = raw_wx;
    float out_y = raw_wy;
    float out_z = raw_wz;

    if (DISPLAY_ORIENTATION == 1) { // Landscape Left (Standard CoD Mobile)
        out_x = -raw_wy;
        out_y = raw_wx;
    } else if (DISPLAY_ORIENTATION == 2) { // Landscape Right
        out_x = raw_wy;
        out_y = -raw_wx;
    }

    final_gyro[0] = out_x * SENSITIVITY;
    final_gyro[1] = out_y * SENSITIVITY;
    final_gyro[2] = out_z * SENSITIVITY;

    last_q = current_q;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        
        for (ssize_t i = 0; i < actual_events; i++) {
            bool state_updated = false;

            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                last_ax = events[i].acceleration.v[0];
                last_ay = events[i].acceleration.v[1];
                last_az = events[i].acceleration.v[2];
                state_updated = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                last_mx = events[i].uncalibrated_magnetic.x_uncalib - HARD_IRON_X;
                last_my = events[i].uncalibrated_magnetic.y_uncalib - HARD_IRON_Y;
                last_mz = events[i].uncalibrated_magnetic.z_uncalib - HARD_IRON_Z;
                state_updated = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) {
                last_mx = events[i].magnetic.v[0];
                last_my = events[i].magnetic.v[1];
                last_mz = events[i].magnetic.v[2];
                state_updated = true;
            } 

            // 3. Timestamp-based synchronization
            if (state_updated) {
                if (unified_ts == 0) {
                    unified_ts = events[i].timestamp;
                } else {
                    float dt = (events[i].timestamp - unified_ts) / 1000000000.0f;
                    if (dt > 0.001f && dt < 0.1f) {
                        update_orientation(dt);
                        derive_angular_velocity(dt);
                    }
                    unified_ts = events[i].timestamp;
                }
            }

            // Output the continuous, synchronized angular velocity
            if (events[i].type == ASENSOR_TYPE_GYROSCOPE) {
                events[i].vector.x = final_gyro[0]; 
                events[i].vector.y = final_gyro[1]; 
                events[i].vector.z = final_gyro[2]; 
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
