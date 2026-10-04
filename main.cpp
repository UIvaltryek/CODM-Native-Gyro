#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
// Proportional gain: Controls how aggressively the quaternion tracks your hands.
// Higher = faster response (lower delay). Lower = smoother macro movements.
// Recommended gaming range: 8.0f - 16.0f
static const float MAHONY_KP = 10.0f; 

// Master Sensitivity multiplier (1.0f = pure 1:1 hardware translation)
static const float SENSITIVITY = 1.0f;

// Kills resting hardware sensor noise floor
static const float NOISE_FLOOR = 0.003f;

// --- STATE VARIABLES ---
// Quaternion orientation state (w, x, y, z) initialized to identity
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

static float last_accel[3] = {0.0f, 0.0f, 9.81f};
static float last_mag[3]   = {0.0f, 1.0f, 0.0f};
static int64_t last_timestamp = 0;

static float final_gyro[3] = {0.0f, 0.0f, 0.0f};

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;

typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

// Clean, NEON-optimized inverse square root
inline float inv_sqrt(float x) {
    return 1.0f / sqrtf(x);
}

// Pure Quaternion Mahony AHRS Algorithm (Gyro-less Architecture)
void mahony_update(float ax, float ay, float az, float mx, float my, float mz, float dt) {
    // 1. Vector Normalization
    float norm_a = inv_sqrt(ax * ax + ay * ay + az * az);
    if (isnan(norm_a) || isinf(norm_a)) return;
    ax *= norm_a; ay *= norm_a; az *= norm_a;

    float norm_m = inv_sqrt(mx * mx + my * my + mz * mz);
    if (isnan(norm_m) || isinf(norm_m)) return;
    mx *= norm_m; my *= norm_m; mz *= norm_m;

    // Auxiliary variables to minimize arithmetic ops per frame
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // 2. Continuous 3D Tilt-Compensation
    // Rotate measured magnetic vector into the Earth coordinate frame (h)
    float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    float hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    float bx = sqrtf(hx * hx + hy * hy);
    float bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // 3. Estimated Gravity (v) and Magnetic Field (w) half-vectors in device body frame
    float halfvx = q1q3 - q0q2;
    float halfvy = q0q1 + q2q3;
    float halfvz = q0q0 - 0.5f + q3q3;

    float halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    float halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    float halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // 4. Calculate 3D Cross-Product Error Torque: e = (a x v) + (m x w)
    float halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    float halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    float halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // 5. Angular Velocity Feedback Vector (rad/s)
    // Scale the raw half-error torque by proportional gain
    float wx = 2.0f * MAHONY_KP * halfex;
    float wy = 2.0f * MAHONY_KP * halfey;
    float wz = 2.0f * MAHONY_KP * halfez;

    // 6. Integrate Quaternion Rate: q_dot = 0.5 * q * omega
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * wx - qc * wy - q3 * wz) * (0.5f * dt);
    q1 += ( qa * wx + qc * wz - q3 * wy) * (0.5f * dt);
    q2 += ( qa * wy - qb * wz + q3 * wx) * (0.5f * dt);
    q3 += ( qa * wz + qb * wy - qc * wx) * (0.5f * dt);

    // Normalize final quaternion
    float norm_q = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm_q; q1 *= norm_q; q2 *= norm_q; q3 *= norm_q;

    // 7. Inject Direct Angular Velocity into Game Payload
    final_gyro[0] = (fabsf(wx) > NOISE_FLOOR) ? wx * SENSITIVITY : 0.0f;
    final_gyro[1] = (fabsf(wy) > NOISE_FLOOR) ? wy * SENSITIVITY : 0.0f;
    final_gyro[2] = (fabsf(wz) > NOISE_FLOOR) ? wz * SENSITIVITY : 0.0f;
}

void compute_sensor_fusion(int64_t timestamp) {
    if (last_timestamp == 0) {
        last_timestamp = timestamp;
        return;
    }

    float dt = (timestamp - last_timestamp) / 1000000000.0f;
    // Guard against timestamp batch anomalies and divide-by-zero spikes
    if (dt <= 0.001f || dt > 0.1f) {
        last_timestamp = timestamp;
        return;
    }

    mahony_update(
        last_accel[0], last_accel[1], last_accel[2],
        last_mag[0],   last_mag[1],   last_mag[2],
        dt
    );

    last_timestamp = timestamp;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        bool fusion_needed = false;
        int64_t latest_ts = 0;

        for (ssize_t i = 0; i < actual_events; i++) {
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                last_accel[0] = events[i].acceleration.v[0];
                last_accel[1] = events[i].acceleration.v[1];
                last_accel[2] = events[i].acceleration.v[2];
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
            } 
            // Intercept Uncalibrated Magnetometer (Type 14) to bypass OS hard-iron drift
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                last_mag[0] = events[i].uncalibrated_magnetic.x_uncalib;
                last_mag[1] = events[i].uncalibrated_magnetic.y_uncalib;
                last_mag[2] = events[i].uncalibrated_magnetic.z_uncalib;
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
            } 
            // Fallback for drivers that route Type 2 only
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) {
                last_mag[0] = events[i].magnetic.v[0];
                last_mag[1] = events[i].magnetic.v[1];
                last_mag[2] = events[i].magnetic.v[2];
                if (events[i].timestamp > latest_ts) latest_ts = events[i].timestamp;
                fusion_needed = true;
            } 
            // Trojan horse delivery: Overwrite stock virtual gyro payload
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE) {
                events[i].vector.x = final_gyro[0]; 
                events[i].vector.y = final_gyro[1]; 
                events[i].vector.z = final_gyro[2]; 
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

void install_hook() {
    void* libandroid = dlopen("libandroid.so", RTLD_NOW);
    if (libandroid) {
        void* target_get = dlsym(libandroid, "ASensorEventQueue_getEvents");
        if (target_get) {
            DobbyHook(target_get, (dobby_dummy_func_t)hook_ASensorEventQueue_getEvents, (dobby_dummy_func_t*)&orig_getEvents);
        }
        void* target_rate = dlsym(libandroid, "ASensorEventQueue_setEventRate");
        if (target_rate) {
            DobbyHook(target_rate, (dobby_dummy_func_t)hook_ASensorEventQueue_setEventRate, (dobby_dummy_func_t*)&orig_setEventRate);
        }
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
