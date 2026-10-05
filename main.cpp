#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- HARDWARE CALIBRATION ---
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

// --- STATE TRACKERS ---
static float accel_data[3] = {0.0f, 0.0f, 9.81f};
static float mag_data[3] = {0.0f, 1.0f, 0.0f};

// We track the last normalized vectors to calculate the derivative (Rate of Change)
static float last_A[3] = {0.0f, 0.0f, 1.0f};
static float last_M[3] = {0.0f, 1.0f, 0.0f};

static float final_gyro[3] = {0.0f, 0.0f, 0.0f};
static int64_t prev_ts = 0;
static bool is_initialized = false;

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

// THE PATCHUP: Pure Vector Derivative Kinematics
void compute_algebraic_kinematics(int64_t timestamp) {
    if (!is_initialized) {
        prev_ts = timestamp;
        is_initialized = true;
        return;
    }

    float dt = (timestamp - prev_ts) / 1000000000.0f;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.016f; // Fallback to 60Hz frame time

    // 1. Normalize Accelerometer (A)
    float ax = accel_data[0], ay = accel_data[1], az = accel_data[2];
    float norm_a = sqrt(ax*ax + ay*ay + az*az);
    if (norm_a > 0.001f) { ax /= norm_a; ay /= norm_a; az /= norm_a; }

    // 2. Normalize Magnetometer (M)
    float mx = mag_data[0], my = mag_data[1], mz = mag_data[2];
    float norm_m = sqrt(mx*mx + my*my + mz*mz);
    if (norm_m > 0.001f) { mx /= norm_m; my /= norm_m; mz /= norm_m; }

    // 3. Calculate Vector Derivatives (How fast are the sensors physically changing?)
    float dAx = (ax - last_A[0]) / dt;
    float dAy = (ay - last_A[1]) / dt;
    float dAz = (az - last_A[2]) / dt;

    float dMx = (mx - last_M[0]) / dt;
    float dMy = (my - last_M[1]) / dt;
    float dMz = (mz - last_M[2]) / dt;

    // ==========================================
    // EQUATION 1: Pitch & Roll (A × dA)
    // Extracts velocity strictly from Gravity tilting
    // ==========================================
    float wx_perp = ay * dAz - az * dAy;
    float wy_perp = az * dAx - ax * dAz;
    float wz_perp = ax * dAy - ay * dAx;

    // ==========================================
    // EQUATION 2: Yaw (Magnetic Sweep)
    // Formula: A * [ A · (M × dM) / |A × M|^2 ]
    // Funnels magnetic sweeping perfectly onto the gravity axis.
    // ==========================================
    
    // Part A: (M × dM)
    float m_cross_x = my * dMz - mz * dMy;
    float m_cross_y = mz * dMx - mx * dMz;
    float m_cross_z = mx * dMy - my * dMx;

    // Part B: A · (M × dM)
    float dot_A_Mdot = ax * m_cross_x + ay * m_cross_y + az * m_cross_z;

    // Part C: |A × M|^2 
    float Hx = ay * mz - az * my;
    float Hy = az * mx - ax * mz;
    float Hz = ax * my - ay * mx;
    float mag_H2 = Hx*Hx + Hy*Hy + Hz*Hz;

    // Calculate the Yaw Scalar multiplier (added 0.000001f to prevent division by zero at magnetic poles)
    float yaw_scalar = dot_A_Mdot / (mag_H2 + 0.000001f);

    // ==========================================
    // FINAL FUSION
    // ==========================================
    final_gyro[0] = wx_perp + (ax * yaw_scalar);
    final_gyro[1] = wy_perp + (ay * yaw_scalar);
    final_gyro[2] = wz_perp + (az * yaw_scalar);

    // Save states for next frame derivative calculation
    last_A[0] = ax; last_A[1] = ay; last_A[2] = az;
    last_M[0] = mx; last_M[1] = my; last_M[2] = mz;
    prev_ts = timestamp;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        bool fusion_ready = false;
        int64_t latest_ts = 0;

        for (ssize_t i = 0; i < actual_events; i++) {
            
            // 1. Gather raw hardware data
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                accel_data[0] = events[i].acceleration.x;
                accel_data[1] = events[i].acceleration.y;
                accel_data[2] = events[i].acceleration.z;
                latest_ts = events[i].timestamp;
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                mag_data[0] = events[i].uncalibrated_magnetic.x_uncalib - HARD_IRON_X;
                mag_data[1] = events[i].uncalibrated_magnetic.y_uncalib - HARD_IRON_Y;
                mag_data[2] = events[i].uncalibrated_magnetic.z_uncalib - HARD_IRON_Z;
                if (!fusion_ready) latest_ts = events[i].timestamp;
                fusion_ready = true;
            }
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) {
                mag_data[0] = events[i].magnetic.x;
                mag_data[1] = events[i].magnetic.y;
                mag_data[2] = events[i].magnetic.z;
                if (!fusion_ready) latest_ts = events[i].timestamp;
                fusion_ready = true;
            }
            
            // 2. Feed our direct kinematic velocity to the game
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE || events[i].type == 4 || events[i].type == 16) {
                events[i].vector.x = final_gyro[0]; 
                events[i].vector.y = final_gyro[1]; 
                events[i].vector.z = final_gyro[2]; 
            }
        }

        // Fire the physics calculation
        if (fusion_ready && latest_ts > 0) {
            compute_algebraic_kinematics(latest_ts);
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
