#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- MAHONY AHRS TUNING ---
// Kp: Proportional gain (Higher = snaps to Accelerometer/Magnetometer faster)
// Ki: Integral gain (Higher = eliminates steady-state drift faster)
static const float Kp = 2.0f; 
static const float Ki = 0.005f; 
static const float MAG_NOISE_GATE = 0.015f; 

// --- FACTORY BIASES (MT6835 Hard-Iron Offsets) ---
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

static float final_gyro[3] = {0.0f, 0.0f, 0.0f}; 

// Sensor States
static float last_accel[3] = {0.0f, 0.0f, 9.81f}; 
static float last_mag[3] = {0.0f, 1.0f, 0.0f}; 
static int64_t last_timestamp = 0; 

static float last_pitch = 0.0f; 
static float last_roll = 0.0f; 
static float last_hx = 0.0f; 
static float last_hy = 1.0f; 
static float last_hz = 0.0f; 

// Mahony States
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float eIntX = 0.0f, eIntY = 0.0f, eIntZ = 0.0f;

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t); 
static getEvents_t orig_getEvents = nullptr; 
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

// --- MAHONY AHRS UPDATE (Modified to output compensated Gyro) ---
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt, float& out_gx, float& out_gy, float& out_gz) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalize accelerometer
        recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        // Normalize magnetometer
        recipNorm = 1.0f / sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
        q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
        q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is cross product between estimated and measured direction of vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback
        if(Ki > 0.0f) {
            eIntX += Ki * halfex * dt;
            eIntY += Ki * halfey * dt;
            eIntZ += Ki * halfez * dt;
            gx += eIntX; gy += eIntY; gz += eIntZ;
        }

        // Apply proportional feedback
        gx += Kp * halfex;
        gy += Kp * halfey;
        gz += Kp * halfez;
    }

    // Capture the flawlessly error-corrected gyroscope data before it integrates to the quaternion!
    out_gx = gx;
    out_gy = gy;
    out_gz = gz;

    // Integrate rate of change to maintain the Quaternion filter foundation
    gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt);
    qa = q0; qb = q1; qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalize quaternion
    recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

void compute_sensor_fusion(int64_t timestamp) {
    float ax = last_accel[0]; float ay = last_accel[1]; float az = last_accel[2]; 
    float mx = last_mag[0]; float my = last_mag[1]; float mz = last_mag[2]; 

    float G = sqrt(ax*ax + ay*ay + az*az); 
    if (G < 0.1f) G = 0.1f; 

    float gx = ax / G; float gy = ay / G; float gz = az / G; 

    // Angular Position Extraction
    float pitch = atan2(ay, az); 
    float normalized_ax = -ax / G; 
    if (normalized_ax > 1.0f) normalized_ax = 1.0f; 
    if (normalized_ax < -1.0f) normalized_ax = -1.0f; 
    float roll = asin(normalized_ax); 

    // Horizontal Magnetic Plane Extraction
    float dot_mg = mx * gx + my * gy + mz * gz; 
    float hx = mx - dot_mg * gx; 
    float hy = my - dot_mg * gy; 
    float hz = mz - dot_mg * gz; 

    float H = sqrt(hx*hx + hy*hy + hz*hz); 
    if (H < 0.01f) H = 0.01f; 
    hx /= H; hy /= H; hz /= H; 

    if (last_timestamp == 0) { 
        last_timestamp = timestamp; 
        last_pitch = pitch; last_roll = roll; 
        last_hx = hx; last_hy = hy; last_hz = hz; 
        return; 
    }

    float raw_dt = (timestamp - last_timestamp) / 1000000000.0f; 
    if (raw_dt <= 0.0f || raw_dt > 0.1f) { 
        last_timestamp = timestamp; 
        return; 
    }

    // 1. RAW ANGULAR VELOCITIES (Your Method)
    float delta_pitch = pitch - last_pitch;
    while (delta_pitch > M_PI) delta_pitch -= 2.0f * M_PI;
    while (delta_pitch < -M_PI) delta_pitch += 2.0f * M_PI;
    float raw_wx = delta_pitch / raw_dt; 

    float delta_roll = roll - last_roll;
    while (delta_roll > M_PI) delta_roll -= 2.0f * M_PI;
    while (delta_roll < -M_PI) delta_roll += 2.0f * M_PI;
    float raw_wy = delta_roll / raw_dt; 

    // Cross-Product Yaw Extraction (Anti-Snap Fix)
    float cx = last_hy * hz - last_hz * hy; 
    float cy = last_hz * hx - last_hx * hz; 
    float cz = last_hx * hy - last_hy * hx; 
    float sin_yaw = cx * gx + cy * gy + cz * gz; 
    if (sin_yaw > 1.0f) sin_yaw = 1.0f;
    if (sin_yaw < -1.0f) sin_yaw = -1.0f;
    float mag_delta_yaw = asin(sin_yaw); 
    
    float raw_wz = mag_delta_yaw / raw_dt; 
    if (fabs(raw_wz) < MAG_NOISE_GATE) raw_wz = 0.0f; 

    // 2. MAHONY ERROR-CORRECTION FUSION
    float clean_wx, clean_wy, clean_wz;
    MahonyAHRSupdate(raw_wx, raw_wy, raw_wz, ax, ay, az, mx, my, mz, raw_dt, clean_wx, clean_wy, clean_wz);

    // 3. PURE DIRECT ADDITION (No Fade Factor, Strict Z-Axis Cull)
    final_gyro[0] = clean_wx; 
    final_gyro[1] = clean_wy + clean_wz; 
    final_gyro[2] = 0.0f; 

    // Update States
    last_pitch = pitch; last_roll = roll; 
    last_hx = hx; last_hy = hy; last_hz = hz; 
    last_timestamp = timestamp; 
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        bool fusion_ready = false;
        int64_t latest_ts = 0;

        for (ssize_t i = 0; i < actual_events; i++) {
            latest_ts = events[i].timestamp;
            
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                last_accel[0] = events[i].acceleration.x;
                last_accel[1] = events[i].acceleration.y;
                last_accel[2] = events[i].acceleration.z;
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                last_mag[0] = events[i].uncalibrated_magnetic.x_uncalib - HARD_IRON_X;
                last_mag[1] = events[i].uncalibrated_magnetic.y_uncalib - HARD_IRON_Y;
                last_mag[2] = events[i].uncalibrated_magnetic.z_uncalib - HARD_IRON_Z;
                fusion_ready = true;
            }
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE || events[i].type == 4 || events[i].type == 16) {
                events[i].vector.x = final_gyro[0]; 
                events[i].vector.y = final_gyro[1]; 
                events[i].vector.z = final_gyro[2]; 
            }
        }

        if (fusion_ready && latest_ts > 0) {
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
