#include <android/sensor.h>
#include <android/looper.h>
#include <dlfcn.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <atomic>
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
//    is exactly the rotation of that attitude over that sample's own interval -> whatever the game
//    integrates stays locked to the attitude estimate (no random walk, no drift).
//  * Sensor events are in the device's natural frame; the game does the landscape remap, as with a real gyro.
//  * The game rarely subscribes to the magnetometer, so the module opens its OWN accel+mag queue on a private
//    thread (started on the game's first getEvents call). If that fails it falls back to whatever accel/mag
//    events the game's queue carries, and without any magnetometer it still tracks tilt.
//
// Debug:  adb logcat -s VGYRO     (one line per second for the first 30 s)
// Tuning: gl::Params below (tilt_hz / yaw_hz = loop bandwidths; higher = less lag, more jitter).
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
    float mag_wait = 0.6f;    // s: no magnetometer by then => start tilt-only (turning is invisible without it); re-anchors when it shows up
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
        have_a = have_m = inited = c0_ok = anchored = false; t_last = t_out = ta = tm = first_a = 0;
        wa = wm = 0.f; g0 = 9.80665f; m0 = 0.f; c0 = 0.f; mcount = 0;
    }

    void onAccel(V3 a, int64_t t) {
        advance(t);
        float n = len(a); if (!(n > 1e-3f)) return;
        if (!first_a) first_a = t;
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
        if (inited && !anchored && wm > 0.5f) anchor_yaw();
    }

    // Angular velocity (rad/s, device frame, right-hand rule) over (previous sample, t]. Call once per gyro event.
    V3 sample(int64_t t) {
        if (!inited) { advance(t); if (!inited) return {0.f, 0.f, 0.f}; }
        if (t > t_last) { if ((t - t_last) * 1e-9f < 0.05f) advance(t); else t = t_last; }   // coast up to 50 ms; longer silence => output stops
        if (!inited) return {0.f, 0.f, 0.f};
        float dt = (t - t_out) * 1e-9f;
        if (dt < 1e-6f) return {0.f, 0.f, 0.f};                          // nothing new: never repeat an old rate
        Quat qt = q_at(t);                                               // the filter may be ahead of this gyro event or behind it
        last_out = qlog(qmul(qconj(q_out), qt)) * (1.f / dt);
        q_out = qt; t_out = t;
        return last_out;
    }

    bool  ready()      const { return inited; }
    bool  has_heading() const { return inited && anchored; }
    float rate_now()   const { return len(last_out); }
    float tilt_trust() const { return wa; }
    float yaw_trust()  const { return wm; }
    Quat  attitude()   const { return q; }

private:
    Params P;
    Quat q, q_out; V3 w, last_out, ua, um;
    struct Hist { int64_t t; Quat q; };
    enum { HN = 256 };                                              // ~0.6 s of attitude history at 2.5 ms steps
    Hist hist[HN]; int hcount = 0, hhead = 0;
    void hist_reset(int64_t t) { hcount = 0; hhead = 0; hist_push(t, q); }
    void hist_push(int64_t t, Quat qq) { hhead = (hhead + 1) % HN; hist[hhead] = {t, qq}; if (hcount < HN) hcount++; }
    Quat q_at(int64_t t) const {                                    // attitude at time t (interpolated, shortest arc)
        if (hcount == 0 || t >= hist[hhead].t) return q;
        for (int n = 1; n < hcount; n++) {
            int j = (hhead - n + HN) % HN;
            if (hist[j].t <= t) {
                const Hist& a = hist[j]; const Hist& b = hist[(j + 1) % HN];
                if (b.t <= a.t) return a.q;
                float f = (float)((double)(t - a.t) / (double)(b.t - a.t));
                Quat qb = b.q; if (a.q.w * qb.w + a.q.x * qb.x + a.q.y * qb.y + a.q.z * qb.z < 0.f) qb = {-qb.w, -qb.x, -qb.y, -qb.z};
                return qnorm({a.q.w + (qb.w - a.q.w) * f, a.q.x + (qb.x - a.q.x) * f, a.q.y + (qb.y - a.q.y) * f, a.q.z + (qb.z - a.q.z) * f});
            }
        }
        return hist[(hhead - hcount + 1 + HN) % HN].q;              // older than the history: oldest we have
    }
    bool have_a, have_m, inited, c0_ok, anchored;
    int64_t t_last, t_out, ta, tm, first_a;
    float wa, wm, g0, m0, c0; int mcount;

    void init_attitude(int64_t t, bool with_mag) {
        V3 n;
        if (with_mag) {
            V3 h = um - ua * dot(um, ua); float hn = len(h);
            if (hn < 0.05f) return;                                 // field parallel to gravity: heading undefined
            n = h * (1.f / hn);
        } else {                                                    // no magnetometer: any horizontal direction will do
            V3 ref = fabsf(ua.z) < 0.9f ? V3{0.f, 0.f, 1.f} : V3{0.f, 1.f, 0.f};
            V3 h = ref - ua * dot(ref, ua); n = h * (1.f / len(h));
        }
        q = quat_from_rows(n, cross(ua, n), ua);                    // rows: north, west, up
        w = {0.f, 0.f, 0.f}; q_out = q; t_last = t_out = t; inited = true; last_out = {0.f, 0.f, 0.f};
        anchored = with_mag; hist_reset(t);
        if (with_mag) { c0 = dot(um, ua); c0_ok = true; } else c0_ok = false;
    }

    // Magnetometer arrived after a tilt-only start: adopt its heading silently (no rotation is reported to the game).
    void anchor_yaw() {
        V3 up = up_in_dev(q); V3 h = um - up * dot(um, up); float hn = len(h);
        if (hn < 0.1f) return;
        V3 n = h * (1.f / hn);
        q = quat_from_rows(n, cross(up, n), up); q_out = q; t_out = t_last; hist_reset(t_last);
        c0 = dot(um, up); c0_ok = true; anchored = true;
    }

    void advance(int64_t t) {
        if (!inited) {
            if (have_a && have_m) init_attitude(t, true);
            else if (have_a && first_a && (t - first_a) * 1e-9f > P.mag_wait) init_attitude(t, false);
            return;
        }
        if (t <= t_last) return;
        float dt = (t - t_last) * 1e-9f;
        if (dt > P.max_gap) { inited = false; have_a = have_m = c0_ok = anchored = false; mcount = 0; first_a = 0; return; }   // restart after a pause
        int n = (int)ceilf(dt / 0.0025f); float h = dt / n;
        int64_t t0 = t_last;
        for (int i = 0; i < n; i++) {
            int64_t ts = (i == n - 1) ? t : t0 + (int64_t)((i + 1) * h * 1e9f);
            step(h, ts); hist_push(ts, q);
        }
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

// ---- sensor access: the hook on the game's queue + our OWN accel/mag subscription ----
// Most games never subscribe to the magnetometer, so we cannot rely on the game's queue to carry it.
static const float HARD_IRON_X = 93.76f;      // your magnetometer offsets (uT), used for the uncalibrated mag type
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

static gl::VirtualGyro vg;                    // default tuning lives in gl::Params
static std::mutex vg_lock;                    // game thread and our sensor thread both touch it

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static getEvents_t orig_getEvents = nullptr;
static setEventRate_t orig_setEventRate = nullptr;

// ---- logging, resolved at runtime (no extra link dependency):  adb logcat -s VGYRO ----
typedef int (*log_print_t)(int, const char*, const char*, ...);
static log_print_t log_print = nullptr;
#define VLOG(...) do { if (log_print) log_print(4, "VGYRO", __VA_ARGS__); } while (0)
static void log_init() { if (!log_print) log_print = (log_print_t)dlsym(RTLD_DEFAULT, "__android_log_print"); }

static int c_game_acc = 0, c_game_mag = 0, c_game_gyro = 0, c_own_acc = 0, c_own_mag = 0;   // guarded by vg_lock
static std::atomic<int> own_state{0};                                                       // 0 off, 1 running, -1 failed/stopped

static inline void feed_accel(const ASensorEvent& e) {
    vg.onAccel({e.acceleration.x, e.acceleration.y, e.acceleration.z}, e.timestamp);
}
static inline void feed_mag_uncal(const ASensorEvent& e) {
    vg.onMag({e.uncalibrated_magnetic.x_uncalib - HARD_IRON_X,
              e.uncalibrated_magnetic.y_uncalib - HARD_IRON_Y,
              e.uncalibrated_magnetic.z_uncalib - HARD_IRON_Z}, e.timestamp);
}
static inline void feed_mag_cal(const ASensorEvent& e) {
    vg.onMag({e.magnetic.x, e.magnetic.y, e.magnetic.z}, e.timestamp);
}

// ---- our own sensor queue (accelerometer + magnetometer) on a private thread ----
struct NdkApi {
    ASensorManager* (*getInstanceForPackage)(const char*) = nullptr;
    ASensorManager* (*getInstance)() = nullptr;
    const ASensor* (*getDefaultSensor)(ASensorManager*, int) = nullptr;
    ALooper* (*looperPrepare)(int) = nullptr;
    ASensorEventQueue* (*createEventQueue)(ASensorManager*, ALooper*, int, ALooper_callbackFunc, void*) = nullptr;
    int (*enableSensor)(ASensorEventQueue*, const ASensor*) = nullptr;
    int (*setEventRate)(ASensorEventQueue*, const ASensor*, int32_t) = nullptr;
    int (*pollOnce)(int, int*, int*, void**) = nullptr;
    ssize_t (*getEvents)(ASensorEventQueue*, ASensorEvent*, size_t) = nullptr;
};

static bool load_ndk(NdkApi& a) {
    void* lib = dlopen("libandroid.so", RTLD_NOW);
    if (!lib) return false;
    a.getInstanceForPackage = (decltype(a.getInstanceForPackage))dlsym(lib, "ASensorManager_getInstanceForPackage");
    a.getInstance           = (decltype(a.getInstance))dlsym(lib, "ASensorManager_getInstance");
    a.getDefaultSensor      = (decltype(a.getDefaultSensor))dlsym(lib, "ASensorManager_getDefaultSensor");
    a.looperPrepare         = (decltype(a.looperPrepare))dlsym(lib, "ALooper_prepare");
    a.createEventQueue      = (decltype(a.createEventQueue))dlsym(lib, "ASensorManager_createEventQueue");
    a.enableSensor          = (decltype(a.enableSensor))dlsym(lib, "ASensorEventQueue_enableSensor");
    a.pollOnce              = (decltype(a.pollOnce))dlsym(lib, "ALooper_pollOnce");
    if (!a.pollOnce) a.pollOnce = (decltype(a.pollOnce))dlsym(lib, "ALooper_pollAll");
    // use the un-hooked originals for our own queue
    a.getEvents   = orig_getEvents ? orig_getEvents : (decltype(a.getEvents))dlsym(lib, "ASensorEventQueue_getEvents");
    a.setEventRate = orig_setEventRate ? orig_setEventRate : (decltype(a.setEventRate))dlsym(lib, "ASensorEventQueue_setEventRate");
    return (a.getInstanceForPackage || a.getInstance) && a.getDefaultSensor && a.looperPrepare && a.createEventQueue &&
           a.enableSensor && a.setEventRate && a.pollOnce && a.getEvents;
}
static bool (*ndk_loader)(NdkApi&) = load_ndk;       // replaceable in tests

#define VG_LOOPER_ID 3                               // LOOPER_ID_USER

static void* sensor_thread_main(void* arg) {
    NdkApi api = *(NdkApi*)arg; delete (NdkApi*)arg;
    char pkg[256] = {0};
    if (FILE* f = fopen("/proc/self/cmdline", "r")) { size_t n = fread(pkg, 1, sizeof(pkg) - 1, f); pkg[n] = 0; fclose(f); }
    ASensorManager* mgr = (api.getInstanceForPackage && pkg[0]) ? api.getInstanceForPackage(pkg) : nullptr;
    if (!mgr && api.getInstance) mgr = api.getInstance();
    if (!mgr) { VLOG("own subscription: no ASensorManager"); own_state.store(-1); return nullptr; }
    ALooper* looper = api.looperPrepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    ASensorEventQueue* q = looper ? api.createEventQueue(mgr, looper, VG_LOOPER_ID, nullptr, nullptr) : nullptr;
    const ASensor* acc = api.getDefaultSensor(mgr, ASENSOR_TYPE_ACCELEROMETER);
    int mag_type = 14;
    const ASensor* mag = api.getDefaultSensor(mgr, 14);                  // uncalibrated (your offsets apply)
    if (!mag) { mag_type = 2; mag = api.getDefaultSensor(mgr, 2); }      // else calibrated
    if (!q || !acc || !mag) { VLOG("own subscription: queue=%d accel=%d mag=%d", q != nullptr, acc != nullptr, mag != nullptr); own_state.store(-1); return nullptr; }
    if (api.enableSensor(q, acc) < 0 || api.enableSensor(q, mag) < 0) { VLOG("own subscription: enableSensor failed"); own_state.store(-1); return nullptr; }
    api.setEventRate(q, acc, 5000);                                      // 200 Hz (no special permission needed)
    api.setEventRate(q, mag, 10000);                                     // 100 Hz
    own_state.store(1);
    VLOG("own subscription running: accel + %s magnetometer", mag_type == 14 ? "uncalibrated" : "calibrated");
    ASensorEvent ev[64];
    for (;;) {
        int ident = api.pollOnce(500, nullptr, nullptr, nullptr);
        if (ident == ALOOPER_POLL_ERROR) break;
        if (ident != VG_LOOPER_ID) continue;
        ssize_t n;
        while ((n = api.getEvents(q, ev, 64)) > 0) {
            std::lock_guard<std::mutex> lock(vg_lock);
            for (ssize_t i = 0; i < n; i++) {
                if (ev[i].type == ASENSOR_TYPE_ACCELEROMETER) { c_own_acc++; feed_accel(ev[i]); }
                else if (mag_type == 14 && ev[i].type == 14)  { c_own_mag++; feed_mag_uncal(ev[i]); }
                else if (mag_type == 2 && ev[i].type == 2)    { c_own_mag++; feed_mag_cal(ev[i]); }
            }
        }
    }
    VLOG("own subscription stopped");
    own_state.store(-1);
    return nullptr;
}

static std::atomic_flag own_started = ATOMIC_FLAG_INIT;
static void start_sensor_thread() {
    NdkApi* api = new NdkApi();
    if (!ndk_loader(*api)) { VLOG("own subscription: could not resolve libandroid symbols"); own_state.store(-1); delete api; return; }
    pthread_t th;
    if (pthread_create(&th, nullptr, sensor_thread_main, api) == 0) pthread_detach(th);
    else { VLOG("own subscription: pthread_create failed"); own_state.store(-1); delete api; }
}

// ---- once a second (first 30 s), then every 10 s: what is actually happening ----
static void maybe_log() {                          // call with vg_lock held
    if (!log_print) return;
    static double next = 0; static int logs = 0;
    timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    double now = ts.tv_sec + ts.tv_nsec * 1e-9;
    if (now < next) return;
    next = now + (logs++ < 30 ? 1.0 : 10.0);
    int os = own_state.load();
    VLOG("game queue: acc=%d mag=%d gyro=%d | own sub: %s acc=%d mag=%d | ready=%d heading=%d trust tilt=%.2f yaw=%.2f | |w|=%.3f rad/s",
         c_game_acc, c_game_mag, c_game_gyro, os == 1 ? "on" : (os < 0 ? "FAILED" : "off"), c_own_acc, c_own_mag,
         (int)vg.ready(), (int)vg.has_heading(), vg.tilt_trust(), vg.yaw_trust(), vg.rate_now());
    c_game_acc = c_game_mag = c_game_gyro = c_own_acc = c_own_mag = 0;
}

// ---- hook on the game's queue: rewrite its gyro events ----
ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t n = orig_getEvents(queue, events, count);
    if (!own_started.test_and_set()) { log_init(); VLOG("first getEvents call: starting own accel+mag subscription"); start_sensor_thread(); }
    if (n <= 0) return n;
    const bool own = own_state.load() == 1;        // if our own subscription is up, ignore accel/mag in the game's queue
    std::lock_guard<std::mutex> lock(vg_lock);
    for (ssize_t i = 0; i < n; i++) {
        ASensorEvent& e = events[i];
        if (e.type == ASENSOR_TYPE_ACCELEROMETER) {
            c_game_acc++; if (!own) feed_accel(e);
        } else if (e.type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || e.type == 14) {
            c_game_mag++; if (!own) feed_mag_uncal(e);
        } else if (e.type == ASENSOR_TYPE_GYROSCOPE || e.type == 4 || e.type == 16) {
            c_game_gyro++;
            gl::V3 w = vg.sample(e.timestamp);          // rotation of the attitude since the previous gyro sample
            e.vector.x = w.x; e.vector.y = w.y; e.vector.z = w.z;
        }
    }
    maybe_log();
    return n;
}

// ---- setEventRate hook (unchanged behaviour: ask for the fastest rate) ----
int hook_ASensorEventQueue_setEventRate(ASensorEventQueue* queue, ASensor const* sensor, int32_t usec) {
    return orig_setEventRate(queue, sensor, 0);      // Android 12+ may cap this without HIGH_SAMPLING_RATE_SENSORS
}

void install_hook() {
    log_init();
    void* libandroid = dlopen("libandroid.so", RTLD_NOW);
    if (libandroid) {
        void* target_get = dlsym(libandroid, "ASensorEventQueue_getEvents");
        void* target_rate = dlsym(libandroid, "ASensorEventQueue_setEventRate");
        // resolve before hooking so our own thread always has the un-hooked originals
        if (target_get) DobbyHook(target_get, (dobby_dummy_func_t)hook_ASensorEventQueue_getEvents, (dobby_dummy_func_t*)&orig_getEvents);
        if (target_rate) DobbyHook(target_rate, (dobby_dummy_func_t)hook_ASensorEventQueue_setEventRate, (dobby_dummy_func_t*)&orig_setEventRate);
        VLOG("hooks installed: getEvents=%d setEventRate=%d", target_get != nullptr, target_rate != nullptr);
    } else {
        VLOG("dlopen(libandroid.so) failed - no hooks");
    }
}

class GyroModifier : public zygisk::ModuleBase {
public:
    void onLoad(zygisk::Api* api, JNIEnv* env) override { this->api = api; this->env = env; }
    void preAppSpecialize(zygisk::AppSpecializeArgs* args) override {
        const char* process = env->GetStringUTFChars(args->nice_name, nullptr);
        if (process) {
            static const char* const targets[] = { "com.activision.callofduty.shooter",   // CODM (global)
                                                   "com.garena.game.codm",                 // CODM (Garena)
                                                   "com.tencent.tmgp.cod",                 // CODM (Tencent)
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