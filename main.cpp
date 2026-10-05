#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

// --- INDEPENDENT STATE TRACKERS ---
static float last_A[3] = {0.0f, 0.0f, 1.0f};
static float last_M[3] = {0.0f, 1.0f, 0.0f};

// We now store the physical speeds independently so they don't overwrite each other with zeroes
static float current_dA[3] = {0.0f, 0.0f, 0.0f};
static float current_dM[3] = {0.0f, 0.0f, 0.0f};

static int64_t last_accel_ts = 0;
static int64_t last_mag_ts = 0;
static bool accel_init = false;
static bool mag_init = false;

static float final_gyro[3] = {0.0f, 0.0f, 0.0f};

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

void compute_algebraic_fusion() {
    if (!accel_init || !mag_init) return;

    // Pitch & Roll (Gravity Tilting) uses current_dA
    float wx_perp = last_A[1] * current_dA[2] - last_A[2] * current_dA[1];
    float wy_perp = last_A[2] * current_dA[0] - last_A[0] * current_dA[2];
    float wz_perp = last_A[0] * current_dA[1] - last_A[1] * current_dA[0];

    // Yaw (Magnetic Sweeping) uses current_dM
    float m_cross_x = last_M[1] * current_dM[2] - last_M[2] * current_dM[1];
    float m_cross_y = last_M[2] * current_dM[0] - last_M[0] * current_dM[2];
    float m_cross_z = last_M[0] * current_dM[1] - last_M[1] * current_dM[0];

    float dot_A_Mdot = last_A[0] * m_cross_x + last_A[1] * m_cross_y + last_A[2] * m_cross_z;

    float Hx = last_A[1] * last_M[2] - last_A[2] * last_M[1];
    float Hy = last_A[2] * last_M[0] - last_A[0] * last_M[2];
    float Hz = last_A[0] * last_M[1] - last_A[1] * last_M[0];
    float mag_H2 = Hx*Hx + Hy*Hy + Hz*Hz;

    float yaw_scalar = dot_A_Mdot / (mag_H2 + 0.000001f);

    // Final velocity output mapped to the correct hardware axis
    final_gyro[0] = wx_perp + (last_A[0] * yaw_scalar);
    final_gyro[1] = wy_perp + (last_A[1] * yaw_scalar);
    final_gyro[2] = wz_perp + (last_A[2] * yaw_scalar);
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        bool needs_fusion = false;

        for (ssize_t i = 0; i < actual_events; i++) {
            
            // --- ACCELEROMETER LOOP ---
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                float ax = events[i].acceleration.x;
                float ay = events[i].acceleration.y;
                float az = events[i].acceleration.z;
                
                float norm_a = sqrt(ax*ax + ay*ay + az*az);
                if (norm_a > 0.001f) { ax /= norm_a; ay /= norm_a; az /= norm_a; }

                if (!accel_init) {
                    last_A[0] = ax; last_A[1] = ay; last_A[2] = az;
                    last_accel_ts = events[i].timestamp;
                    accel_init = true;
                    continue;
                }

                float dt = (events[i].timestamp - last_accel_ts) / 1000000000.0f;
                if (dt > 0.0001f && dt < 0.1f) {
                    current_dA[0] = (ax - last_A[0]) / dt;
                    current_dA[1] = (ay - last_A[1]) / dt;
                    current_dA[2] = (az - last_A[2]) / dt;
                    
                    last_A[0] = ax; last_A[1] = ay; last_A[2] = az;
                    last_accel_ts = events[i].timestamp;
                    needs_fusion = true;
                }
            } 
            
            // --- MAGNETOMETER LOOP ---
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14 || events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) {
                float mx, my, mz;
                if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) {
                    mx = events[i].magnetic.x; my = events[i].magnetic.y; mz = events[i].magnetic.z;
                } else {
                    mx = events[i].uncalibrated_magnetic.x_uncalib - HARD_IRON_X;
                    my = events[i].uncalibrated_magnetic.y_uncalib - HARD_IRON_Y;
                    mz = events[i].uncalibrated_magnetic.z_uncalib - HARD_IRON_Z;
                }
                
                float norm_m = sqrt(mx*mx + my*my + mz*mz);
                if (norm_m > 0.001f) { mx /= norm_m; my /= norm_m; mz /= norm_m; }

                if (!mag_init) {
                    last_M[0] = mx; last_M[1] = my; last_M[2] = mz;
                    last_mag_ts = events[i].timestamp;
                    mag_init = true;
                    continue;
                }

                float dt = (events[i].timestamp - last_mag_ts) / 1000000000.0f;
                if (dt > 0.0001f && dt < 0.1f) {
                    current_dM[0] = (mx - last_M[0]) / dt;
                    current_dM[1] = (my - last_M[1]) / dt;
                    current_dM[2] = (mz - last_M[2]) / dt;
                    
                    last_M[0] = mx; last_M[1] = my; last_M[2] = mz;
                    last_mag_ts = events[i].timestamp;
                    needs_fusion = true;
                }
            }
        }

        // Run the math only if one of the sensors actually updated this batch
        if (needs_fusion) {
            compute_algebraic_fusion();
        }

        // Inject the cleanly fused gyro into the game's stream
        for (ssize_t i = 0; i < actual_events; i++) {
            if (events[i].type == ASENSOR_TYPE_GYROSCOPE || events[i].type == 4 || events[i].type == 16) {
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
