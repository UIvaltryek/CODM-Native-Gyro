#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// Madgwick Beta (Filter Gain). 
// Lower = smoother but slower. Higher = faster but picks up more noise. 0.1f is standard.
static const float BETA = 0.1f; 

static float output_gyro[3] = {0.0f, 0.0f, 0.0f};

static float last_accel[3] = {0.0f, 0.0f, 9.81f};
static float last_mag[3] = {0.0f, 1.0f, 0.0f};
static int64_t last_timestamp = 0;

// 4D Quaternion State (w, x, y, z)
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;
typedef ASensor const* (*getDefaultSensor_t)(ASensorManager*, int);
static getDefaultSensor_t orig_getDefaultSensor = nullptr;
typedef ASensor const* (*getDefaultSensorEx_t)(ASensorManager*, int, bool);
static getDefaultSensorEx_t orig_getDefaultSensorEx = nullptr;

void compute_madgwick_vgyro(int64_t timestamp) {
    if (last_timestamp == 0) {
        last_timestamp = timestamp;
        return;
    }

    float dt = (timestamp - last_timestamp) / 1000000000.0f; 
    if (dt <= 0.001f || dt > 0.1f) { 
        last_timestamp = timestamp;
        return;
    }
    last_timestamp = timestamp;

    float ax = last_accel[0], ay = last_accel[1], az = last_accel[2];
    float mx = last_mag[0], my = last_mag[1], mz = last_mag[2];

    // Normalize accelerometer measurement
    float normA = sqrt(ax * ax + ay * ay + az * az);
    if (normA == 0.0f) return; 
    ax /= normA; ay /= normA; az /= normA;

    // Normalize magnetometer measurement
    float normM = sqrt(mx * mx + my * my + mz * mz);
    if (normM == 0.0f) return;
    mx /= normM; my /= normM; mz /= normM;

    // Reference direction of Earth's magnetic field
    float hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) + my * (q1 * q2 - q0 * q3) + mz * (q1 * q3 + q0 * q2));
    float hy = 2.0f * (mx * (q1 * q2 + q0 * q3) + my * (0.5f - q1 * q1 - q3 * q3) + mz * (q2 * q3 - q0 * q1));
    float bx = sqrt(hx * hx + hy * hy);
    float bz = 2.0f * (mx * (q1 * q3 - q0 * q2) + my * (q2 * q3 + q0 * q1) + mz * (0.5f - q1 * q1 - q2 * q2));

    // Gradient descent algorithm corrective step
    float s0 = -_2q2 * (2.0f * q1 * q3 - _2q0 * q2 - ax) + _2q1 * (2.0f * q0 * q1 + _2q2 * q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2 * q2 - q3 * q3) + _2bz * (q1 * q3 - q0 * q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3) - my) + _2bx * q2 * (_2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1 * q1 - q2 * q2) - mz);
    float s1 = _2q3 * (2.0f * q1 * q3 - _2q0 * q2 - ax) + _2q0 * (2.0f * q0 * q1 + _2q2 * q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1 * q1 - 2.0f * q2 * q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2 * q2 - q3 * q3) + _2bz * (q1 * q3 - q0 * q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1 * q1 - q2 * q2) - mz);
    float s2 = -_2q0 * (2.0f * q1 * q3 - _2q0 * q2 - ax) + _2q3 * (2.0f * q0 * q1 + _2q2 * q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1 * q1 - 2.0f * q2 * q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2 * q2 - q3 * q3) + _2bz * (q1 * q3 - q0 * q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1 * q1 - q2 * q2) - mz);
    float s3 = _2q1 * (2.0f * q1 * q3 - _2q0 * q2 - ax) + _2q2 * (2.0f * q0 * q1 + _2q2 * q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2 * q2 - q3 * q3) + _2bz * (q1 * q3 - q0 * q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3) - my) + _2bx * q1 * (_2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1 * q1 - q2 * q2) - mz);

    // Normalize step magnitude
    float normS = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (normS > 0.0f) {
        s0 /= normS; s1 /= normS; s2 /= normS; s3 /= normS;
    }

    // Backup current quaternion to calculate delta
    float last_q0 = q0, last_q1 = q1, last_q2 = q2, last_q3 = q3;

    // Apply gradient descent step to find the new absolute orientation quaternion
    q0 -= BETA * s0 * dt;
    q1 -= BETA * s1 * dt;
    q2 -= BETA * s2 * dt;
    q3 -= BETA * s3 * dt;

    // Normalize new quaternion
    float normQ = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= normQ; q1 /= normQ; q2 /= normQ; q3 /= normQ;

    // Calculate Delta Quaternion (Hamiltonian Conjugate Product) to extract Virtual Gyro Rates
    float dq0 = last_q0 * q0 + last_q1 * q1 + last_q2 * q2 + last_q3 * q3;
    float dq1 = last_q0 * q1 - last_q1 * q0 - last_q2 * q3 + last_q3 * q2;
    float dq2 = last_q0 * q2 + last_q1 * q3 - last_q2 * q0 - last_q3 * q1;
    float dq3 = last_q0 * q3 - last_q1 * q2 + last_q2 * q1 - last_q3 * q0;

    // Enforce shortest path
    if (dq0 < 0.0f) {
        dq1 = -dq1; dq2 = -dq2; dq3 = -dq3;
    }

    // Convert pure quaternion derivative to rad/s (w = 2 * dq / dt)
    float speed_x = (2.0f * dq1) / dt;
    float speed_y = (2.0f * dq2) / dt;
    float speed_z = (2.0f * dq3) / dt;

    // Globally inverted outputs to match Unity's Right-Handed Camera mapping
    output_gyro[0] = -speed_x;
    output_gyro[1] = -speed_y;
    output_gyro[2] = -speed_z;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        bool fusion_needed = false;
        int64_t latest_ts = 0;

        for (ssize_t i = 0; i < actual_events; i++) {
            if (events[i].type == 35 || events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                last_accel[0] = events[i].data[0];
                last_accel[1] = events[i].data[1];
                last_accel[2] = events[i].data[2];
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
                if (events[i].type == 35) events[i].type = ASENSOR_TYPE_ACCELEROMETER; 
            } 
            else if (events[i].type == 14 || events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) {
                last_mag[0] = events[i].data[0];
                last_mag[1] = events[i].data[1];
                last_mag[2] = events[i].data[2];
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
                if (events[i].type == 14) events[i].type = ASENSOR_TYPE_MAGNETIC_FIELD; 
            }
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE) {
                events[i].vector.x = output_gyro[0]; 
                events[i].vector.y = output_gyro[1]; 
                events[i].vector.z = output_gyro[2]; 
            }
        }
        if (fusion_needed && latest_ts > 0) compute_madgwick_vgyro(latest_ts);
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
