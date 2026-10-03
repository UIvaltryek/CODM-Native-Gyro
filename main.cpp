#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// The electronic brake to stop the "springy rubberband" effect on flicks
static const float MAG_NOISE_GATE = 0.01f; 

static float output_gyro[3] = {0.0f, 0.0f, 0.0f};

static float last_accel[3] = {0.0f, 0.0f, 9.81f};
static float last_mag[3] = {0.0f, 1.0f, 0.0f};
static int64_t last_timestamp = 0;

// 4D Quaternion State (w, x, y, z)
static float last_qw = 1.0f;
static float last_qx = 0.0f;
static float last_qy = 0.0f;
static float last_qz = 0.0f;

// Fallback for East vector singularity
static float last_ex = 1.0f, last_ey = 0.0f, last_ez = 0.0f;

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;
typedef ASensor const* (*getDefaultSensor_t)(ASensorManager*, int);
static getDefaultSensor_t orig_getDefaultSensor = nullptr;
typedef ASensor const* (*getDefaultSensorEx_t)(ASensorManager*, int, bool);
static getDefaultSensorEx_t orig_getDefaultSensorEx = nullptr;

void compute_sensor_fusion(int64_t timestamp) {
    float ax = last_accel[0], ay = last_accel[1], az = last_accel[2];
    float mx = last_mag[0], my = last_mag[1], mz = last_mag[2];

    // 1. Up Vector (Gravity)
    float normA = sqrt(ax*ax + ay*ay + az*az);
    if (normA < 0.001f) normA = 0.001f; 
    float ux = ax / normA, uy = ay / normA, uz = az / normA;

    // 2. East Vector (Magnetic cross Gravity)
    float ex = my * uz - mz * uy;
    float ey = mz * ux - mx * uz;
    float ez = mx * uy - my * ux;
    float normE = sqrt(ex*ex + ey*ey + ez*ez);
    if (normE < 0.001f) {
        ex = last_ex; ey = last_ey; ez = last_ez; 
    } else {
        ex /= normE; ey /= normE; ez /= normE;
        last_ex = ex; last_ey = ey; last_ez = ez;
    }

    // 3. North Vector (Gravity cross East)
    float nx = uy * ez - uz * ey;
    float ny = uz * ex - ux * ez;
    float nz = ux * ey - uy * ex;

    // 4. Matrix to Quaternion Conversion (w, x, y, z)
    float qw, qx, qy, qz;
    float tr = ex + ny + uz; 

    if (tr > 0.0f) {
        float S = sqrt(tr + 1.0f) * 2.0f; 
        qw = 0.25f * S;
        qx = (nz - uy) / S;
        qy = (ux - ez) / S;
        qz = (ey - nx) / S;
    } else if ((ex > ny) && (ex > uz)) {
        float S = sqrt(1.0f + ex - ny - uz) * 2.0f; 
        qw = (nz - uy) / S;
        qx = 0.25f * S;
        qy = (nx + ey) / S;
        qz = (ux + ez) / S;
    } else if (ny > uz) {
        float S = sqrt(1.0f + ny - ex - uz) * 2.0f; 
        qw = (ux - ez) / S;
        qx = (nx + ey) / S;
        qy = 0.25f * S;
        qz = (uy + nz) / S;
    } else {
        float S = sqrt(1.0f + uz - ex - ny) * 2.0f; 
        qw = (ey - nx) / S;
        qx = (ux + ez) / S;
        qy = (uy + nz) / S;
        qz = 0.25f * S;
    }

    // Normalize Quaternion
    float qNorm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    qw /= qNorm; qx /= qNorm; qy /= qNorm; qz /= qNorm;

    if (last_timestamp == 0) {
        last_qw = qw; last_qx = qx; last_qy = qy; last_qz = qz;
        last_timestamp = timestamp;
        return;
    }

    float raw_dt = (timestamp - last_timestamp) / 1000000000.0f; 
    if (raw_dt <= 0.001f || raw_dt > 0.1f) { 
        last_timestamp = timestamp;
        return;
    }

    // 5. Delta Quaternion (Inverse Last * Current)
    float dw = last_qw * qw + last_qx * qx + last_qy * qy + last_qz * qz;
    float dx = last_qw * qx - last_qx * qw - last_qy * qz + last_qz * qy;
    float dy = last_qw * qy + last_qx * qz - last_qy * qw - last_qz * qx;
    float dz = last_qw * qz - last_qx * qy + last_qy * qx - last_qz * qw;

    // Enforce shortest rotational path
    if (dw < 0.0f) {
        dx = -dx; dy = -dy; dz = -dz;
    }

    // 6. Convert to pure rad/s
    float speed_x = (2.0f * dx) / raw_dt;
    float speed_y = (2.0f * dy) / raw_dt;
    float speed_z = (2.0f * dz) / raw_dt;

    // Noise gates
    if (fabs(speed_x) < MAG_NOISE_GATE) speed_x = 0.0f;
    if (fabs(speed_y) < MAG_NOISE_GATE) speed_y = 0.0f;
    if (fabs(speed_z) < MAG_NOISE_GATE) speed_z = 0.0f;

    output_gyro[0] = speed_x;
    output_gyro[1] = speed_y;
    output_gyro[2] = speed_z;

    last_qw = qw; last_qx = qx; last_qy = qy; last_qz = qz;
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

        if (fusion_needed && latest_ts > 0) compute_sensor_fusion(latest_ts);
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
