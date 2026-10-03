#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

static const float MAG_NOISE_GATE = 0.01f; 

static float output_gyro[3] = {0.0f, 0.0f, 0.0f};
static float last_accel[3] = {0.0f, 0.0f, 9.81f};
static float last_mag[3] = {0.0f, 1.0f, 0.0f};
static int64_t last_timestamp = 0;

// Replaced 2D Euler states with 3D Orthogonal Basis Vectors
static float last_zx = 0.0f, last_zy = 0.0f, last_zz = 1.0f;
static float last_ex = 1.0f, last_ey = 0.0f, last_ez = 0.0f;
static float last_nx = 0.0f, last_ny = 1.0f, last_nz = 0.0f;

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;
typedef ASensor const* (*getDefaultSensor_t)(ASensorManager*, int);
static getDefaultSensor_t orig_getDefaultSensor = nullptr;
typedef ASensor const* (*getDefaultSensorEx_t)(ASensorManager*, int, bool);
static getDefaultSensorEx_t orig_getDefaultSensorEx = nullptr;

void compute_sensor_fusion(int64_t timestamp) {
    float ax = last_accel[0]; float ay = last_accel[1]; float az = last_accel[2];
    float mx = last_mag[0]; float my = last_mag[1]; float mz = last_mag[2];

    // 1. Z-Basis (Gravity / Down Vector)
    float G = sqrt(ax*ax + ay*ay + az*az);
    if (G < 0.1f) G = 0.1f; 
    float zx = ax / G; float zy = ay / G; float zz = az / G;

    // 2. E-Basis (East Vector) = Magnetic Field cross Gravity
    float ex = my * zz - mz * zy;
    float ey = mz * zx - mx * zz;
    float ez = mx * zy - my * zx;
    float E = sqrt(ex*ex + ey*ey + ez*ez);
    if (E < 0.01f) E = 0.01f;
    ex /= E; ey /= E; ez /= E;

    // 3. N-Basis (North Vector) = Gravity cross East
    float nx = zy * ez - zz * ey;
    float ny = zz * ex - zx * ez;
    float nz = zx * ey - zy * ex;

    if (last_timestamp == 0) {
        last_zx = zx; last_zy = zy; last_zz = zz;
        last_ex = ex; last_ey = ey; last_ez = ez;
        last_nx = nx; last_ny = ny; last_nz = nz;
        last_timestamp = timestamp;
        return;
    }

    float raw_dt = (timestamp - last_timestamp) / 1000000000.0f; 
    if (raw_dt <= 0.001f || raw_dt > 0.1f) { 
        last_timestamp = timestamp;
        return;
    }

    // Measure how fast the basis vectors are changing in 3D space
    float dzx = (zx - last_zx) / raw_dt;
    float dzy = (zy - last_zy) / raw_dt;
    float dzz = (zz - last_zz) / raw_dt;

    float dex = (ex - last_ex) / raw_dt;
    float dey = (ey - last_ey) / raw_dt;
    float dez = (ez - last_ez) / raw_dt;

    // KINEMATICS: Absolute Rotational Velocity (rad/s)
    // By dot-multiplying the changes against the old vectors, we extract perfect gyro data
    // completely immune to Gimbal Lock, 90-degree inversions, and boundary wrapping.
    float speed_pitch = (last_nx * dzx + last_ny * dzy + last_nz * dzz);
    float speed_roll  = -(last_ex * dzx + last_ey * dzy + last_ez * dzz);
    float speed_yaw   = -(last_nx * dex + last_ny * dey + last_nz * dez);

    // Your original custom landscape blend
    float pitch = atan2(ay, az); 
    float fade_factor = pow(fabs(sin(pitch)), 4.0f);
    float final_horizontal_speed = (speed_roll * (1.0f - fade_factor)) + (speed_yaw * fade_factor);

    // Apply strict noise gates to kill micro-vibrations
    if (fabs(speed_pitch) < MAG_NOISE_GATE) speed_pitch = 0.0f;
    if (fabs(final_horizontal_speed) < MAG_NOISE_GATE) final_horizontal_speed = 0.0f;

    output_gyro[0] = speed_pitch;
    output_gyro[1] = final_horizontal_speed;
    output_gyro[2] = 0.0f;

    // Save states for next frame
    last_zx = zx; last_zy = zy; last_zz = zz;
    last_ex = ex; last_ey = ey; last_ez = ez;
    last_nx = nx; last_ny = ny; last_nz = nz;
    last_timestamp = timestamp;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        bool fusion_needed = false;
        int64_t latest_ts = 0;

        for (ssize_t i = 0; i < actual_events; i++) {
            
            // Standard Calibrated Fallback
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                last_accel[0] = events[i].acceleration.v[0];
                last_accel[1] = events[i].acceleration.v[1];
                last_accel[2] = events[i].acceleration.v[2];
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
            
            // Raw Uncalibrated Accelerometer (Type 35)
            else if (events[i].type == 35) {
                last_accel[0] = events[i].data[0];
                last_accel[1] = events[i].data[1];
                last_accel[2] = events[i].data[2];
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
                events[i].type = ASENSOR_TYPE_ACCELEROMETER; 
            }
            
            // Raw Uncalibrated Compass (Type 14)
            else if (events[i].type == 14) {
                last_mag[0] = events[i].data[0];
                last_mag[1] = events[i].data[1];
                last_mag[2] = events[i].data[2];
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
                events[i].type = ASENSOR_TYPE_MAGNETIC_FIELD; 
            }
            
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE) {
                events[i].vector.x = output_gyro[0]; 
                events[i].vector.y = output_gyro[1]; 
                events[i].vector.z = output_gyro[2]; 
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

ASensor const* hook_ASensorManager_getDefaultSensor(ASensorManager* manager, int type) {
    if (type == ASENSOR_TYPE_ACCELEROMETER) {
        ASensor const* uncal_accel = orig_getDefaultSensor(manager, 35); 
        if (uncal_accel) return uncal_accel;
    }
    if (type == ASENSOR_TYPE_MAGNETIC_FIELD) {
        ASensor const* uncal_mag = orig_getDefaultSensor(manager, 14); 
        if (uncal_mag) return uncal_mag;
    }
    return orig_getDefaultSensor(manager, type);
}

ASensor const* hook_ASensorManager_getDefaultSensorEx(ASensorManager* manager, int type, bool wakeUp) {
    if (type == ASENSOR_TYPE_ACCELEROMETER) {
        ASensor const* uncal_accel = orig_getDefaultSensorEx(manager, 35, wakeUp);
        if (uncal_accel) return uncal_accel;
    }
    if (type == ASENSOR_TYPE_MAGNETIC_FIELD) {
        ASensor const* uncal_mag = orig_getDefaultSensorEx(manager, 14, wakeUp);
        if (uncal_mag) return uncal_mag;
    }
    return orig_getDefaultSensorEx(manager, type, wakeUp);
}

void install_hook() {
    void* libandroid = dlopen("libandroid.so", RTLD_NOW);
    if (libandroid) {
        void* target_get = dlsym(libandroid, "ASensorEventQueue_getEvents");
        if (target_get) DobbyHook(target_get, (dobby_dummy_func_t)hook_ASensorEventQueue_getEvents, (dobby_dummy_func_t*)&orig_getEvents);
        
        void* target_rate = dlsym(libandroid, "ASensorEventQueue_setEventRate");
        if (target_rate) DobbyHook(target_rate, (dobby_dummy_func_t)hook_ASensorEventQueue_setEventRate, (dobby_dummy_func_t*)&orig_setEventRate);
        
        void* target_default = dlsym(libandroid, "ASensorManager_getDefaultSensor");
        if (target_default) DobbyHook(target_default, (dobby_dummy_func_t)hook_ASensorManager_getDefaultSensor, (dobby_dummy_func_t*)&orig_getDefaultSensor);
        
        void* target_default_ex = dlsym(libandroid, "ASensorManager_getDefaultSensorEx");
        if (target_default_ex) DobbyHook(target_default_ex, (dobby_dummy_func_t)hook_ASensorManager_getDefaultSensorEx, (dobby_dummy_func_t*)&orig_getDefaultSensorEx);
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
