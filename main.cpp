#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
static const float NOISE_GATE = 0.005f; 
static const float SENSITIVITY = 1.0f; // Use -1.0f if axes are inverted

// Exact CPU X Factory Biases for MT6835 - Perfectly centers the horizontal spin
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

// --- DECOUPLED STATE TRACKERS ---
// Pitch (Vertical Aim) State
static float last_pitch = 0.0f;
static int64_t last_accel_ts = 0;
static float speed_pitch = 0.0f;
static Vec3 current_gravity = {0.0f, 0.0f, 1.0f};

// Yaw (Horizontal Aim) State
static Vec3 last_east = {1.0f, 0.0f, 0.0f};
static int64_t last_mag_ts = 0;
static float speed_yaw = 0.0f;

static float final_gyro[3] = {0.0f, 0.0f, 0.0f};

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

// 1. Process Vertical Aim (200Hz Accelerometer)
void process_accel(float ax, float ay, float az, int64_t ts) {
    current_gravity = normalize_vec({ax, ay, az});
    
    // Calculate simple 2D pitch angle based on Y and Z gravity components
    float pitch = atan2(ay, az);
    
    if (last_accel_ts != 0) {
        float dt = (ts - last_accel_ts) / 1000000000.0f;
        if (dt > 0.001f && dt < 0.1f) {
            float delta_pitch = pitch - last_pitch;
            
            // Handle wrap-around (e.g., flipping phone upside down)
            if (delta_pitch > M_PI) delta_pitch -= 2.0f * M_PI;
            if (delta_pitch < -M_PI) delta_pitch += 2.0f * M_PI;
            
            speed_pitch = delta_pitch / dt;
        }
    }
    last_pitch = pitch;
    last_accel_ts = ts;
}

// 2. Process Horizontal Aim (50Hz Magnetometer -> Upsampled to 200Hz)
void process_mag(float mx, float my, float mz, int64_t ts) {
    // Subtract hardware bias to permanently fix the soft-iron wobble
    Vec3 M = normalize_vec({mx - HARD_IRON_X, my - HARD_IRON_Y, mz - HARD_IRON_Z});
    
    // Create the East vector (Projecting Mag purely flat against Gravity)
    Vec3 E = cross_product(M, current_gravity);
    float len_E = sqrt(E.x*E.x + E.y*E.y + E.z*E.z);
    
    // Singularity guard: Only process if phone isn't pointing perfectly at Magnetic North while flat
    if (len_E > 0.01f) {
        E.x /= len_E; E.y /= len_E; E.z /= len_E;
        
        if (last_mag_ts != 0) {
            float dt = (ts - last_mag_ts) / 1000000000.0f;
            if (dt > 0.001f && dt < 0.2f) {
                // Measure how fast East spun around the Gravity vector
                float yaw_sin = dot_product(cross_product(last_east, E), current_gravity);
                
                if (yaw_sin > 1.0f) yaw_sin = 1.0f;
                if (yaw_sin < -1.0f) yaw_sin = -1.0f;
                
                speed_yaw = asin(yaw_sin) / dt;
            }
        }
        last_east = E;
        last_mag_ts = ts;
    }
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        for (ssize_t i = 0; i < actual_events; i++) {
            
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                process_accel(events[i].acceleration.v[0], events[i].acceleration.v[1], events[i].acceleration.v[2], events[i].timestamp);
                
                // Coasting check: If mag hasn't updated in 40ms, slightly decay Yaw to kill snapback
                if (last_mag_ts != 0 && (events[i].timestamp - last_mag_ts) > 40000000LL) {
                    speed_yaw *= 0.85f; 
                }
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                process_mag(events[i].uncalibrated_magnetic.x_uncalib, events[i].uncalibrated_magnetic.y_uncalib, events[i].uncalibrated_magnetic.z_uncalib, events[i].timestamp);
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) {
                process_mag(events[i].magnetic.v[0], events[i].magnetic.v[1], events[i].magnetic.v[2], events[i].timestamp);
            } 
            
            // 3. Directly Feed the 2D Game Logic (No Quaternions)
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE) {
                final_gyro[0] = (fabs(speed_pitch) > NOISE_GATE) ? speed_pitch * SENSITIVITY : 0.0f;
                final_gyro[1] = (fabs(speed_yaw) > NOISE_GATE) ? speed_yaw * SENSITIVITY : 0.0f;
                final_gyro[2] = 0.0f; // Permanently kills racing-game tilt

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
