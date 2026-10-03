#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
// 1. Raw Hardware Smoothing (0.1 to 1.0). 
// Lower = smoother but slight delay. Higher = instant but picks up hand jitter.
static const float VECTOR_SMOOTHING = 0.6f; 

// 2. Master Sensitivity Scaler
static const float SENSITIVITY = 1.0f;

// 3. Absolute Noise Floor (Kills static crosshair drift when phone is on a table)
static const float NOISE_FLOOR = 0.005f;

// --- 3D Vector Math Structs ---
struct Vec3 { float x, y, z; };

Vec3 cross_product(Vec3 a, Vec3 b) {
    return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}
float dot_product(Vec3 a, Vec3 b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
Vec3 normalize(Vec3 v) {
    float len = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (len < 0.0001f) return {0.0f, 0.0f, 0.0f};
    return { v.x/len, v.y/len, v.z/len };
}
Vec3 scale(Vec3 v, float s) {
    return { v.x*s, v.y*s, v.z*s };
}
Vec3 add(Vec3 a, Vec3 b) {
    return { a.x+b.x, a.y+b.y, a.z+b.z };
}

// --- Memory State ---
static Vec3 last_norm_acc = {0.0f, 0.0f, 0.0f};
static Vec3 last_norm_mag = {0.0f, 0.0f, 0.0f};
static int64_t last_acc_ts = 0;
static int64_t last_mag_ts = 0;

static Vec3 current_omega_acc = {0.0f, 0.0f, 0.0f};
static Vec3 current_omega_mag = {0.0f, 0.0f, 0.0f};

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;

typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        for (ssize_t i = 0; i < actual_events; i++) {
            
            // 1. Process Raw Gravity (Pitch & Roll)
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                Vec3 curr_acc = normalize({events[i].vector.x, events[i].vector.y, events[i].vector.z});
                
                if (last_acc_ts != 0) {
                    float dt = (events[i].timestamp - last_acc_ts) / 1000000000.0f;
                    if (dt > 0.001f && dt < 0.1f) {
                        // Apply lightweight EMA filter to kill hardware jitter
                        curr_acc = normalize(add(scale(curr_acc, VECTOR_SMOOTHING), scale(last_norm_acc, 1.0f - VECTOR_SMOOTHING)));
                        
                        // KINEMATICS: Cross product of current x last yields device rotation in rad/s
                        current_omega_acc = scale(cross_product(curr_acc, last_norm_acc), 1.0f / dt);
                    }
                }
                last_norm_acc = curr_acc;
                last_acc_ts = events[i].timestamp;
            }
            
            // 2. Process Raw Magnetic North (Yaw)
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD || events[i].type == 14) {
                Vec3 curr_mag = normalize({events[i].vector.x, events[i].vector.y, events[i].vector.z});
                
                if (last_mag_ts != 0) {
                    float dt = (events[i].timestamp - last_mag_ts) / 1000000000.0f;
                    if (dt > 0.001f && dt < 0.1f) {
                        curr_mag = normalize(add(scale(curr_mag, VECTOR_SMOOTHING), scale(last_norm_mag, 1.0f - VECTOR_SMOOTHING)));
                        current_omega_mag = scale(cross_product(curr_mag, last_norm_mag), 1.0f / dt);
                    }
                }
                last_norm_mag = curr_mag;
                last_mag_ts = events[i].timestamp;
            }
            
            // 3. Output Synthesis (Overwrite the deadzoned Type 4 Gyro)
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE) {
                
                // Dot product projects the magnetic rotation strictly onto the gravity axis (Isolating pure Yaw)
                float yaw_scalar = dot_product(current_omega_mag, last_norm_acc);
                Vec3 isolated_yaw = scale(last_norm_acc, yaw_scalar);
                
                // Combine Gravity-based Pitch/Roll with Magnetic-based Yaw
                Vec3 final_gyro = add(current_omega_acc, isolated_yaw);
                
                // Apply Sensitivity and Noise Gate
                float out_x = (fabs(final_gyro.x) > NOISE_FLOOR) ? final_gyro.x * SENSITIVITY : 0.0f;
                float out_y = (fabs(final_gyro.y) > NOISE_FLOOR) ? final_gyro.y * SENSITIVITY : 0.0f;
                float out_z = (fabs(final_gyro.z) > NOISE_FLOOR) ? final_gyro.z * SENSITIVITY : 0.0f;

                // Overwrite the SCP payload
                events[i].vector.x = out_x;
                events[i].vector.y = out_y;
                events[i].vector.z = out_z;
            }
        }
    }
    return actual_events;
}

int hook_ASensorEventQueue_setEventRate(ASensorEventQueue* queue, ASensor const* sensor, int32_t usec) {
    // Force Android to give us the absolute maximum polling rate for all sensors
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
