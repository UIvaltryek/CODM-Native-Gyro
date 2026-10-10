#include <android/sensor.h>
#include <dlfcn.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <mutex>
#include "dobby.h"
#include "zygisk.hpp"

// =====================================================================================================
// Virtual gyroscope (accelerometer + magnetometer) - quaternion rate observer, no Euler angles anywhere.
//
//  * No gimbal lock / singularity: attitude is a unit quaternion, rates come from rotation vectors.
//    Holding the phone upright in landscape ("TV mode") is just another orientation - nothing special-cased.
//  * Tilt (aiming up/down, rolling) is corrected ONLY by the accelerometer; turning left/right (rotation
//    about gravity) ONLY by the magnetometer, so a magnetic disturbance can never tilt the picture.
//  * The angular velocity is the integral state of a PI observer, and each gyro sample we hand to the game
//    is exactly the rotation of that attitude since the previous sample -> whatever the game integrates stays
//    locked to the attitude estimate (no random walk, no drift).
//  * Sensor events are in the device's natural frame; the game does the landscape remap, as with a real gyro.
//
// Tuning (gl::Params below): tilt_hz / yaw_hz = loop bandwidths. Higher = less lag, more jitter.
// Only rewrites gyro events the game already receives; needs accel + uncalibrated-mag events in the same
// queue (or add your own subscription).
// =====================================================================================================

// ===================== gyroless.inc =====================
// Virtual gyroscope from accelerometer + magnetometer. No Android dependencies, no Euler angles.
//
//   * Attitude is a unit quaternion q (device -> earth; earth axes: x = magnetic north, y = west, z = up).
//   * Accelerometer corrects TILT only, magnetometer corrects YAW (rotation about gravity) only.
//   * The rate estimate is the integral state of a PI attitude observer (a "gyro-less Mahony"):
//         e      = tilt_error + yaw_error                   (both are small-angle rotation vectors, device frame)
//         w_hat += Ki * e * dt                              (angular velocity estimate = what a gyro would measure)
//         q     <- q * exp( (w_hat + Kp * e) * dt / 2 )
//   * Output for each gyro sample = rotation of q since the previous sample / dt, so whatever the game
//     integrates stays locked to q (no random walk, no drift).
namespace gl {

struct V3 { float x, y, z; };
static inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
static inline float len(V3 a) { return sqrtf(dot(a, a)); }
static inline float ramp(float dev, float lo, float hi) {          // 1 below lo, 0 above hi
    float t = (dev - lo) / (hi - lo); return t <= 0.f ? 1.f : (t >= 1.f ? 0.f : 1.f - t);
}

struct Quat { float w, x, y, z; };
static inline Quat qmul(Quat a, Quat b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
static inline Quat qconj(Quat a) { return {a.w, -a.x, -a.y, -a.z}; }
static inline Quat qnorm(Quat a) {
    float n = sqrtf(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
    if (n < 1e-12f) return {1.f, 0.f, 0.f, 0.f};
    float s = 1.f / n; return {a.w * s, a.x * s, a.y * s, a.z * s};
}
static inline Quat qexp(V3 rv) {                                    // rotation vector -> quaternion
    float th = len(rv);
    if (th < 1e-7f) return qnorm({1.f, rv.x * 0.5f, rv.y * 0.5f, rv.z * 0.5f});
    float s = sinf(th * 0.5f) / th; return {cosf(th * 0.5f), rv.x * s, rv.y * s, rv.z * s};
}
static inline V3 qlog(Quat q) {                                     // quaternion -> rotation vector (shortest arc)
    if (q.w < 0.f) q = {-q.w, -q.x, -q.y, -q.z};
    V3 v = {q.x, q.y, q.z}; float s = len(v);
    if (s < 1e-7f) return v * 2.f;
    return v * (2.f * atan2f(s, q.w) / s);
}
// Rows of the device->earth rotation matrix = earth axes expressed in the device frame.
static inline V3 north_in_dev(Quat q) { return {1.f - 2.f * (q.y * q.y + q.z * q.z), 2.f * (q.x * q.y - q.w * q.z), 2.f * (q.x * q.z + q.w * q.y)}; }
static inline V3 up_in_dev(Quat q)    { return {2.f * (q.x * q.z - q.w * q.y), 2.f * (q.w * q.x + q.y * q.z), 1.f - 2.f * (q.x * q.x + q.y * q.y)}; }

static Quat quat_from_rows(V3 r0, V3 r1, V3 r2) {                   // rotation matrix given by its rows
    float R00 = r0.x, R01 = r0.y, R02 = r0.z, R10 = r1.x, R11 = r1.y, R12 = r1.z, R20 = r2.x, R21 = r2.y, R22 = r2.z;
    float tr = R00 + R11 + R22; Quat q;
    if (tr > 0.f)                      { float S = sqrtf(tr + 1.f) * 2.f;                 q = {0.25f * S, (R21 - R12) / S, (R02 - R20) / S, (R10 - R01) / S}; }
    else if (R00 > R11 && R00 > R22)   { float S = sqrtf(1.f + R00 - R11 - R22) * 2.f;   q = {(R21 - R12) / S, 0.25f * S, (R01 + R10) / S, (R02 + R20) / S}; }
    else if (R11 > R22)                { float S = sqrtf(1.f + R11 - R00 - R22) * 2.f;   q = {(R02 - R20) / S, (R01 + R10) / S, 0.25f * S, (R12 + R21) / S}; }
    else                               { float S = sqrtf(1.f + R22 - R00 - R11) * 2.f;   q = {(R10 - R01) / S, (R02 + R20) / S, (R12 + R21) / S, 0.25f * S}; }
    return qnorm(q);
}

struct Params {
    float tilt_hz  = 9.0f;    // bandwidth of the accelerometer loop (aiming up/down, rolling)
    float yaw_hz   = 5.0f;    // bandwidth of the magnetometer loop (turning left/right) - noisiest sensor, keep it lower
    float damping  = 0.9f;
    float leak_s   = 2.0f;    // the rate state forgets with this time constant when nothing corrects it
    float max_rate = 35.0f;   // rad/s clamp on the rate state
    float max_gap  = 0.1f;    // s: longer silence (pause/resume) => restart cleanly
    float stale    = 0.1f;    // s: a held measurement older than this is ignored
    // trust gates: full trust below *_lo, zero trust above *_hi (relative deviations)
    float acc_lo = 0.02f, acc_hi = 0.12f;     // | |a| - 1g | / 1g
    float mag_lo = 0.05f, mag_hi = 0.25f;     // | |m| - usual | / usual
    float dip_lo = 0.04f, dip_hi = 0.15f;     // change of cos(angle between field and gravity)
};

class VirtualGyro {
public:
    explicit VirtualGyro(Params p = Params()) : P(p) { reset(); }

    void reset() {
        q = {1.f, 0.f, 0.f, 0.f}; w = {0.f, 0.f, 0.f}; last_out = {0.f, 0.f, 0.f};
        ua = {0.f, 0.f, 1.f}; um = {0.f, 1.f, 0.f};
        have_a = have_m = inited = c0_ok = false; t_last = t_out = ta = tm = 0;
        wa = wm = 0.f; g0 = 9.80665f; m0 = 0.f; c0 = 0.f; mcount = 0;
    }

    void onAccel(V3 a, int64_t t) {
        advance(t);
        float n = len(a); if (!(n > 1e-3f)) return;
        float dt = ta ? (t - ta) * 1e-9f : 0.f; ta = t;
        g0 += (n - g0) * fminf(1.f, dt / 10.f);                     // slow 1 g reference (tolerates sensor scale error)
        wa = ramp(fabsf(n - g0) / g0, P.acc_lo, P.acc_hi);                // linear acceleration / shaking => trust less, ~0 in free fall
        ua = a * (1.f / n); have_a = true;
    }

    void onMag(V3 m, int64_t t) {
        advance(t);
        float n = len(m); if (!(n > 1e-3f)) return;
        float dt = tm ? (t - tm) * 1e-9f : 0.f; tm = t;
        float k = fmaxf(fminf(1.f, dt / 10.f), 1.f / (1.f + mcount)); if (mcount < 100000) mcount++;
        if (m0 <= 0.f) m0 = n;
        float wmag = ramp(fabsf(n - m0) / m0, P.mag_lo, P.mag_hi);         // field strength must look like the usual field
        m0 += (n - m0) * k;
        V3 u = m * (1.f / n);
        float wdip = 1.f;
        if (inited) {                                               // angle between field and gravity must stay put (dip)
            float c = dot(u, up_in_dev(q));
            if (!c0_ok) { c0 = c; c0_ok = true; }
            wdip = ramp(fabsf(c - c0), P.dip_lo, P.dip_hi);
            c0 += (c - c0) * k;
        }
        wm = wmag * wdip; um = u; have_m = true;
    }

    // Angular velocity (rad/s, device frame, right-hand rule) over (previous sample, t]. Call once per gyro event.
    V3 sample(int64_t t) {
        advance(t);
        if (!inited) return {0.f, 0.f, 0.f};
        float dt = (t_last - t_out) * 1e-9f;
        if (dt < 1e-6f) return last_out;
        last_out = qlog(qmul(qconj(q_out), q)) * (1.f / dt);
        q_out = q; t_out = t_last;
        return last_out;
    }

    float tilt_trust() const { return wa; }
    float yaw_trust()  const { return wm; }
    Quat  attitude()   const { return q; }

private:
    Params P;
    Quat q, q_out; V3 w, last_out, ua, um;
    bool have_a, have_m, inited, c0_ok;
    int64_t t_last, t_out, ta, tm;
    float wa, wm, g0, m0, c0; int mcount;

    void init_attitude(int64_t t) {
        V3 h = um - ua * dot(um, ua); float hn = len(h);
        if (hn < 0.05f) return;                                     // field parallel to gravity: heading undefined
        V3 n = h * (1.f / hn);
        q = quat_from_rows(n, cross(ua, n), ua);                    // rows: north, west, up
        w = {0.f, 0.f, 0.f}; q_out = q; t_last = t_out = t; inited = true; last_out = {0.f, 0.f, 0.f};
        c0 = dot(um, ua); c0_ok = true;                             // field/gravity angle at start = the reference
    }

    void advance(int64_t t) {
        if (!inited) { if (have_a && have_m) init_attitude(t); return; }
        if (t <= t_last) return;
        float dt = (t - t_last) * 1e-9f;
        if (dt > P.max_gap) { inited = false; have_a = have_m = c0_ok = false; mcount = 0; return; }   // restart after a pause
        int n = (int)ceilf(dt / 0.0025f); float h = dt / n;
        for (int i = 0; i < n; i++) step(h, t_last + (int64_t)((i + 1) * h * 1e9f));
        t_last = t;
    }

    void step(float h, int64_t t_now) {
        V3 up = up_in_dev(q), north = north_in_dev(q);
        float wa_e = (have_a && (t_now - ta) * 1e-9f < P.stale) ? wa : 0.f;
        float wm_e = (have_m && (t_now - tm) * 1e-9f < P.stale) ? wm : 0.f;
        V3 e_tilt = cross(ua, up) * wa_e;                           // lies in the plane perpendicular to gravity
        V3 e_yaw = {0.f, 0.f, 0.f};
        V3 hm = um - up * dot(um, up); float hn = len(hm);          // horizontal part of the measured field
        if (hn > 0.1f && wm_e > 0.f) e_yaw = cross(hm * (1.f / hn), north) * wm_e;   // parallel to gravity
        const float wt = 6.2831853f * P.tilt_hz, wy = 6.2831853f * P.yaw_hz;
        const float kp_t = 2.f * P.damping * wt, ki_t = wt * wt, kp_y = 2.f * P.damping * wy, ki_y = wy * wy;
        w = w + (e_tilt * ki_t + e_yaw * ki_y - w * (1.f / P.leak_s)) * h;
        float wl = len(w); if (wl > P.max_rate) w = w * (P.max_rate / wl);
        V3 w_eff = w + e_tilt * kp_t + e_yaw * kp_y;
        q = qnorm(qmul(q, qexp(w_eff * h)));
    }
};

}  // namespace gl

// ---- hook: single pass, events in timestamp order ----
static const float HARD_IRON_X = 93.76f;      // your magnetometer offsets (uT)
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

static gl::VirtualGyro vg;                    // default tuning lives in gl::Params
static std::mutex vg_lock;                    // the game may call getEvents from more than one thread

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t n = orig_getEvents(queue, events, count);
    if (n <= 0) return n;
    std::lock_guard<std::mutex> lock(vg_lock);
    for (ssize_t i = 0; i < n; i++) {
        ASensorEvent& e = events[i];
        if (e.type == ASENSOR_TYPE_ACCELEROMETER) {
            vg.onAccel({e.acceleration.x, e.acceleration.y, e.acceleration.z}, e.timestamp);
        } else if (e.type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || e.type == 14) {
            vg.onMag({e.uncalibrated_magnetic.x_uncalib - HARD_IRON_X,
                      e.uncalibrated_magnetic.y_uncalib - HARD_IRON_Y,
                      e.uncalibrated_magnetic.z_uncalib - HARD_IRON_Z}, e.timestamp);
        } else if (e.type == ASENSOR_TYPE_GYROSCOPE || e.type == 4 || e.type == 16) {
            gl::V3 w = vg.sample(e.timestamp);          // rotation of the attitude since the previous gyro sample
            e.vector.x = w.x; e.vector.y = w.y; e.vector.z = w.z;
        }
    }
    return n;
}

typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

int hook_ASensorEventQueue_setEventRate(ASensorEventQueue* queue, ASensor const* sensor, int32_t usec) {
    return orig_setEventRate(queue, sensor, 0);      // ask for the fastest rate (Android 12+ may cap it without HIGH_SAMPLING_RATE_SENSORS)
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
            static const char* const targets[] = { "com.activision.callofduty.shooter",   // CODM
                                                   "com.tencent.ig",                       // PUBG Mobile
                                                   "com.pubg.imobile" };                   // BGMI
            for (const char* t : targets) if (strcmp(process, t) == 0) enable_hack = true;
            env->ReleaseStringUTFChars(args->nice_name, process);
        }
    }
    void postAppSpecialize(const zygisk::AppSpecializeArgs*) override { if (enable_hack) install_hook(); }
private:
    zygisk::Api* api = nullptr; JNIEnv* env = nullptr; bool enable_hack = false;
};
REGISTER_ZYGISK_MODULE(GyroModifier)