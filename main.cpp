#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h>
#include "dobby.h"
#include "zygisk.hpp"

// Virtual gyro for devices without one: accelerometer (gravity) + magnetometer (heading) -> angular velocity,
// then a Mahony AHRS pass that keeps long-term drift bounded.
//
// Sensor events are always in the device's natural frame (portrait on a phone); the game does the landscape
// remap itself. So we just output the true 3D angular velocity there - orientation independent.
//
//   w_perp (tilt)  = -(g_prev x g_now) / dt                       (g = unit "up" from the accelerometer)
//   w_yaw  (turn)  = -((h_prev x h_now) . g) / dt                 (h = unit horizontal magnetic direction)
//   w              = w_perp + w_yaw * g

static const float Kp = 1.5f;
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;
static const int64_t MIN_SPAN_NS = 8000000;   // minimum baseline for differencing (5-10 ms); shorter = less lag, more jitter

static float final_gyro[3] = {0.0f, 0.0f, 0.0f};
static float last_accel[3] = {0.0f, 0.0f, 9.81f};
static float last_mag[3]   = {0.0f, 1.0f, 0.0f};
static float cur_g[3]      = {0.0f, 0.0f, 1.0f};   // latest unit "up" vector (device frame)
static float ref_g[3], ref_h[3];                   // reference samples we difference against
static int64_t ref_g_ts = 0, ref_h_ts = 0, last_fuse_ts = 0;
static float tilt_w[3] = {0.0f, 0.0f, 0.0f};       // angular rate perpendicular to gravity
static float yaw_w = 0.0f;                         // angular rate about gravity
static bool seeded = false;
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;

void cross_product(float a[3], float b[3], float out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}
void normalize(float v[3]) {
    float norm = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (norm > 0.0001f) { v[0]/=norm; v[1]/=norm; v[2]/=norm; }
}
void seed_mahony(float ax, float ay, float az, float mx, float my, float mz) {
    float A[3] = {ax, ay, az}; normalize(A);
    float M[3] = {mx, my, mz}; normalize(M);
    float E[3]; cross_product(M, A, E); normalize(E);
    float N[3]; cross_product(A, E, N); normalize(N);
    // Mahony's earth frame: x = magnetic north, y = WEST, z = up  ->  columns [N, -E, Up]
    float R[3][3] = { {N[0], -E[0], A[0]}, {N[1], -E[1], A[1]}, {N[2], -E[2], A[2]} };
    float tr = R[0][0] + R[1][1] + R[2][2];
    if (tr > 0.0f) {
        float S = sqrtf(tr + 1.0f) * 2.0f;
        q0 = 0.25f * S; q1 = (R[1][2] - R[2][1]) / S; q2 = (R[2][0] - R[0][2]) / S; q3 = (R[0][1] - R[1][0]) / S;
    } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
        float S = sqrtf(1.0f + R[0][0] - R[1][1] - R[2][2]) * 2.0f;
        q0 = (R[1][2] - R[2][1]) / S; q1 = 0.25f * S; q2 = (R[0][1] + R[1][0]) / S; q3 = (R[2][0] + R[0][2]) / S;
    } else if (R[1][1] > R[2][2]) {
        float S = sqrtf(1.0f + R[1][1] - R[0][0] - R[2][2]) * 2.0f;
        q0 = (R[2][0] - R[0][2]) / S; q1 = (R[0][1] + R[1][0]) / S; q2 = 0.25f * S; q3 = (R[1][2] + R[2][1]) / S;
    } else {
        float S = sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]) * 2.0f;
        q0 = (R[0][1] - R[1][0]) / S; q1 = (R[2][0] + R[0][2]) / S; q2 = (R[1][2] + R[2][1]) / S; q3 = 0.25f * S;
    }
    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if(norm > 0.0001f) { q0/=norm; q1/=norm; q2/=norm; q3/=norm; }
}
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt, float& out_gx, float& out_gy, float& out_gz) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
        recipNorm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;
        q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
        q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
        q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3;
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
        gx += Kp * halfex; gy += Kp * halfey; gz += Kp * halfez;
    }
    out_gx = gx; out_gy = gy; out_gz = gz;
    gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt);
    qa = q0; qb = q1; qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

// ---------- event-driven fusion: every sensor is differenced against ITS OWN timestamps ----------
static void fusion_reset() {
    ref_g_ts = ref_h_ts = 0; seeded = false;
    tilt_w[0] = tilt_w[1] = tilt_w[2] = 0.0f; yaw_w = 0.0f;
    final_gyro[0] = final_gyro[1] = final_gyro[2] = 0.0f;
}

static void fusion_accel(const float a[3], int64_t ts) {
    memcpy(last_accel, a, sizeof last_accel);
    float G = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]); if (G < 0.1f) G = 0.1f;
    float g[3] = { a[0]/G, a[1]/G, a[2]/G };
    memcpy(cur_g, g, sizeof g);
    if (ref_g_ts == 0) { memcpy(ref_g, g, sizeof g); ref_g_ts = ts; return; }
    int64_t span = ts - ref_g_ts;
    if (span < MIN_SPAN_NS) return;
    float dt = span / 1e9f;
    if (dt < 0.1f) {
        float c[3]; cross_product(ref_g, g, c);               // w_perp = -(g_prev x g_now) / dt
        for (int i = 0; i < 3; i++) tilt_w[i] = -c[i] / dt;
    } else { tilt_w[0] = tilt_w[1] = tilt_w[2] = 0.0f; }
    memcpy(ref_g, g, sizeof g); ref_g_ts = ts;
}

static void fusion_mag(const float m[3], int64_t ts) {
    memcpy(last_mag, m, sizeof last_mag);
    if (ref_g_ts == 0) return;                                  // need a gravity sample first
    float d = m[0]*cur_g[0] + m[1]*cur_g[1] + m[2]*cur_g[2];
    float h[3] = { m[0] - d*cur_g[0], m[1] - d*cur_g[1], m[2] - d*cur_g[2] };
    float H = sqrtf(h[0]*h[0] + h[1]*h[1] + h[2]*h[2]);
    if (H < 0.01f) return;
    h[0] /= H; h[1] /= H; h[2] /= H;
    if (ref_h_ts == 0) { memcpy(ref_h, h, sizeof h); ref_h_ts = ts; return; }
    int64_t span = ts - ref_h_ts;
    if (span < MIN_SPAN_NS) return;
    float dt = span / 1e9f;
    if (dt < 0.1f) {
        float c[3]; cross_product(ref_h, h, c);               // rotation about gravity
        yaw_w = -(c[0]*cur_g[0] + c[1]*cur_g[1] + c[2]*cur_g[2]) / dt;
    } else { yaw_w = 0.0f; }
    memcpy(ref_h, h, sizeof h); ref_h_ts = ts;
}

static void fusion_finish(int64_t ts) {
    if (ref_g_ts == 0 || ref_h_ts == 0) return;
    if (!seeded) {
        seed_mahony(last_accel[0], last_accel[1], last_accel[2], last_mag[0], last_mag[1], last_mag[2]);
        seeded = true; last_fuse_ts = ts; return;
    }
    float dt = (ts - last_fuse_ts) / 1e9f;
    last_fuse_ts = ts;
    if (dt <= 0.0f || dt > 0.1f) { fusion_reset(); return; }   // gap (app paused etc): start clean, re-seed
    float w[3] = { tilt_w[0] + yaw_w*cur_g[0], tilt_w[1] + yaw_w*cur_g[1], tilt_w[2] + yaw_w*cur_g[2] };
    MahonyAHRSupdate(w[0], w[1], w[2], last_accel[0], last_accel[1], last_accel[2],
                     last_mag[0], last_mag[1], last_mag[2], dt, final_gyro[0], final_gyro[1], final_gyro[2]);
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t n = orig_getEvents(queue, events, count);
    if (n <= 0) return n;

    int64_t latest = 0;
    for (ssize_t i = 0; i < n; i++) {                          // pass 1: feed fusion in timestamp order
        if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
            fusion_accel(events[i].acceleration.v, events[i].timestamp);
            latest = events[i].timestamp;
        } else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
            float m[3] = { events[i].uncalibrated_magnetic.x_uncalib - HARD_IRON_X,
                           events[i].uncalibrated_magnetic.y_uncalib - HARD_IRON_Y,
                           events[i].uncalibrated_magnetic.z_uncalib - HARD_IRON_Z };
            fusion_mag(m, events[i].timestamp);
            latest = events[i].timestamp;
        }
    }
    if (latest > 0) fusion_finish(latest);

    for (ssize_t i = 0; i < n; i++) {                          // pass 2: gyro events get the FRESH value
        if (events[i].type == ASENSOR_TYPE_GYROSCOPE || events[i].type == 4 || events[i].type == 16) {
            events[i].vector.x = final_gyro[0];
            events[i].vector.y = final_gyro[1];
            events[i].vector.z = final_gyro[2];
        }
    }
    return n;
}

typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

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
        if (process) {
            if (strcmp(process, "com.activision.callofduty.shooter") == 0) { enable_hack = true; }
            env->ReleaseStringUTFChars(args->nice_name, process);
        }
    }
    void postAppSpecialize(const zygisk::AppSpecializeArgs*) override { if (enable_hack) install_hook(); }
private:
    zygisk::Api* api = nullptr; JNIEnv* env = nullptr; bool enable_hack = false;
};
REGISTER_ZYGISK_MODULE(GyroModifier)