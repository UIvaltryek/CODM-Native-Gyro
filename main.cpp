#include <android/sensor.h>
#include <android/looper.h>
#include <dlfcn.h>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/prctl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <atomic>
#include <mutex>
#include "dobby.h"
#include "zygisk.hpp"

namespace gl {

typedef double real;
struct D3 { real x, y, z; };

static inline D3 operator+(D3 a, D3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline D3 operator-(D3 a, D3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline D3 operator*(D3 a, real s) { return {a.x * s, a.y * s, a.z * s}; }
static inline real dot(D3 a, D3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline D3 cross(D3 a, D3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
static inline real len(D3 a) { return sqrt(dot(a, a)); }

struct Quat { real w, x, y, z; };
static inline Quat qmul(Quat a, Quat b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
static inline Quat qnorm(Quat a) {
    real n = sqrt(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
    if (n < 1e-300) return {1.0, 0.0, 0.0, 0.0};
    real s = 1.0 / n; return {a.w * s, a.x * s, a.y * s, a.z * s};
}
static inline Quat qexp(D3 rv) {
    real th = len(rv);
    if (th < 1e-9) return qnorm({1.0, rv.x * 0.5, rv.y * 0.5, rv.z * 0.5});
    real s = sin(th * 0.5) / th; return {cos(th * 0.5), rv.x * s, rv.y * s, rv.z * s};
}

static inline D3 row0(Quat q) { return {1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.w * q.z), 2 * (q.x * q.z + q.w * q.y)}; }
static inline D3 row1(Quat q) { return {2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.w * q.x)}; }
static inline D3 up_in_dev(Quat q) { return {2 * (q.x * q.z - q.w * q.y), 2 * (q.y * q.z + q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y)}; }

static Quat quat_from_rows(D3 r0, D3 r1, D3 r2) {
    real R00 = r0.x, R01 = r0.y, R02 = r0.z, R10 = r1.x, R11 = r1.y, R12 = r1.z, R20 = r2.x, R21 = r2.y, R22 = r2.z;
    real tr = R00 + R11 + R22; Quat q;
    if (tr > 0.0)                      { real S = sqrt(tr + 1.0) * 2.0;                 q = {0.25 * S, (R21 - R12) / S, (R02 - R20) / S, (R10 - R01) / S}; }
    else if (R00 > R11 && R00 > R22)   { real S = sqrt(1.0 + R00 - R11 - R22) * 2.0;   q = {(R21 - R12) / S, 0.25 * S, (R01 + R10) / S, (R02 + R20) / S}; }
    else if (R11 > R22)                { real S = sqrt(1.0 + R11 - R00 - R22) * 2.0;   q = {(R02 - R20) / S, (R01 + R10) / S, 0.25 * S, (R12 + R21) / S}; }
    else                               { real S = sqrt(1.0 + R22 - R00 - R11) * 2.0;   q = {(R10 - R01) / S, (R02 + R20) / S, (R12 + R21) / S, 0.25 * S}; }
    return qnorm(q);
}

static inline D3 rotvec_between(D3 u, D3 v) {
    D3 c = cross(u, v); real s = len(c);
    if (s < 1e-12) return {0.0, 0.0, 0.0};
    return c * (atan2(s, dot(u, v)) / s);
}
static inline real soft_gate(real dev, real tol) { real x = dev / tol; return x >= 1.0 ? 0.0 : 1.0 - x * x; }

struct Params {
    real sigma_tilt  = 0.05;
    real sigma_head  = 0.065;
    real hand_acc    = 70.0;
    real g_tol       = 70.0;
    real mag_tol     = 10.0;
    real dip_tol     = 3.14159;
    real jump_reject = 3.14159;
    real tilt_reject = 3.14159;
    real outage_tau  = 0.15;
    real lock_hz     = 0.0;
    real max_gap     = 1.0;
    real mag_wait    = 0.5;
};

class VirtualGyro {
public:
    std::atomic<float> out_x{0.0f}, out_y{0.0f}, out_z{0.0f};
    std::atomic<int64_t> latest_ts{0};
    std::atomic<uint64_t> seq{0};

    explicit VirtualGyro(Params p = Params()) : P(p) { restart(); }

    void on_accel(D3 a, int64_t t) {
        real n = len(a); if (!(n > 1e-3)) return;
        if (inited) advance(t);
        if (!inited) { a0 = a; ta0 = t; if (!have_a) { have_a = true; ta_first = t; } try_init(t); return; }
        real T = interval(t, t_acc, acc_seen); t_acc = t; acc_seen = true;
        D3 up = up_in_dev(q);
        D3 e = rotvec_between(a * (1.0 / n), up);
        if (len(e) > P.tilt_reject) {
            reject_tilt_t += T;
            if (reject_tilt_t > 0.3) { restart(); a0 = a; ta0 = t; have_a = true; ta_first = t; return; }
            decay(up, T, false); publish(t); return;
        }
        reject_tilt_t = 0.0;
        real wa = soft_gate(fabs(n - G), P.g_tol);
        if (wa < 0.02) { decay(up, T, false); publish(t); return; }
        correct(e, T, P.sigma_tilt / sqrt(wa));
        publish(t);
    }

    void on_mag(D3 m, int64_t t) {
        real nm = len(m); if (!(nm > 1e-3)) return;
        if (inited) advance(t);
        if (!inited) { m0 = m; tm0 = t; have_m = true; try_init(t); return; }
        real T = interval(t, t_mag, mag_seen); t_mag = t; mag_seen = true;
        Quat q0 = q; D3 up = up_in_dev(q0);
        real hx = dot(row0(q0), m), hy = dot(row1(q0), m);
        real hn = sqrt(hx * hx + hy * hy);
        real dip = acos(fmax(-1.0, fmin(1.0, dot(m, up) / nm)));
        if (!ref_valid) {
            if (hn >= 0.2 * nm) { h_refx = hx / hn; h_refy = hy / hn; m_ref = nm; dip_ref = dip; ref_valid = true; }
            return;
        }
        real wgt = soft_gate(fabs(nm - m_ref) / m_ref, P.mag_tol) * soft_gate(fabs(dip - dip_ref), P.dip_tol);
        if (hn < 0.2 * nm) wgt = 0.0;
        real psi = 0.0;
        if (wgt >= 0.02) {
            psi = atan2(hx * h_refy - hy * h_refx, hx * h_refx + hy * h_refy);
            if (fabs(psi) > P.jump_reject) wgt = 0.0;
        }
        if (wgt < 0.02) {
            reject_t += T;
            if (reject_t > 0.5 && hn >= 0.2 * nm) {
                h_refx = hx / hn; h_refy = hy / hn; m_ref = nm; dip_ref = dip; reject_t = 0.0;
            }
            decay(up, T, true); publish(t);
            return;
        }
        reject_t = 0.0;
        correct(up * psi, T, P.sigma_head / sqrt(wgt));
        if (wgt > 0.9) {
            real k = fmin(1.0, T * 0.05);
            m_ref += k * (nm - m_ref); dip_ref += k * (dip - dip_ref);
        }
        publish(t);
    }

private:
    static constexpr real G = 9.80665;
    Params P;
    Quat q{1.0, 0.0, 0.0, 0.0};
    D3 w{0.0, 0.0, 0.0}, a0{0.0, 0.0, 0.0}, m0{0.0, 0.0, 0.0};
    bool inited = false, have_a = false, have_m = false, ref_valid = false, acc_seen = false, mag_seen = false;
    int64_t t_last = 0, t_acc = 0, t_mag = 0, ta0 = 0, tm0 = 0, ta_first = 0;
    real m_ref = 0.0, dip_ref = 0.0, h_refx = 0.0, h_refy = 1.0, reject_t = 0.0, reject_tilt_t = 0.0;

    inline void publish(int64_t t) {
        out_x.store((float)w.x, std::memory_order_relaxed);
        out_y.store((float)w.y, std::memory_order_relaxed);
        out_z.store((float)w.z, std::memory_order_relaxed);
        if (t > latest_ts.load(std::memory_order_relaxed)) {
            latest_ts.store(t, std::memory_order_relaxed);
        }
        seq.fetch_add(1, std::memory_order_release);
    }

    static inline real interval(int64_t t, int64_t prev, bool seen) {
        if (!seen) return 0.005;
        real T = (t - prev) * 1e-9;
        return T < 1e-3 ? 1e-3 : (T > 0.1 ? 0.1 : T);
    }

    inline void correct(D3 e, real T, real sigma_meas) {
        real lam = P.hand_acc * T * T / sigma_meas;
        real r = (4.0 + lam - sqrt(8.0 * lam + lam * lam)) * 0.25;
        real alpha = fmin(1.0 - r * r, 0.92);
        real beta = fmin(fmax(2.0 * (1.0 + r * r) - 4.0 * r, 0.0), 0.85);
        q = qnorm(qmul(q, qexp(e * alpha)));
        w = w + e * (beta / T);
    }

    inline void decay(D3 g, real T, bool along) {
        real k = exp(-T / P.outage_tau);
        D3 par = g * dot(w, g);
        w = along ? (par * k + (w - par)) : (par + (w - par) * k);
    }

    inline void advance(int64_t t) {
        if (t <= t_last) return;
        real dt = (t - t_last) * 1e-9;
        if (dt > P.max_gap) { restart(); return; }
        q = qnorm(qmul(q, qexp(w * dt)));
        t_last = t;
    }

    void restart() {
        inited = have_a = have_m = ref_valid = acc_seen = mag_seen = false;
        w = {0.0, 0.0, 0.0}; reject_t = reject_tilt_t = 0.0;
        out_x.store(0.0f, std::memory_order_relaxed);
        out_y.store(0.0f, std::memory_order_relaxed);
        out_z.store(0.0f, std::memory_order_relaxed);
    }

    void start(int64_t t) {
        inited = true; w = {0.0, 0.0, 0.0}; t_last = t;
        acc_seen = mag_seen = false; reject_t = reject_tilt_t = 0.0;
        publish(t);
    }

    void try_init(int64_t t) {
        if (have_a && have_m) {
            real an = len(a0), mn = len(m0);
            D3 east = cross(m0, a0); real en = len(east);
            if (en < 1e-3 * an * mn) return;
            east = east * (1.0 / en);
            D3 up = a0 * (1.0 / an), north = cross(up, east);
            q = quat_from_rows(east, north, up);
            m_ref = mn; dip_ref = acos(fmax(-1.0, fmin(1.0, dot(m0, a0) / (mn * an))));
            h_refx = 0.0; h_refy = 1.0; ref_valid = true;
            start(ta0 > tm0 ? ta0 : tm0);
        } else if (have_a && (t - ta_first) * 1e-9 > P.mag_wait) {
            real an = len(a0); D3 up = a0 * (1.0 / an);
            D3 ex = {1.0, 0.0, 0.0};
            D3 r0 = ex - up * dot(ex, up);
            if (len(r0) < 0.3) { ex = {0.0, 1.0, 0.0}; r0 = ex - up * dot(ex, up); }
            r0 = r0 * (1.0 / len(r0));
            q = quat_from_rows(r0, cross(up, r0), up);
            ref_valid = false; start(ta0);
        }
    }
};

}  // namespace gl

// =====================================================================================================
// SYNTHETIC 0-DELAY PACKET GENERATOR & HARDWARE ACCEL CLOCK HIJACKING
// =====================================================================================================

static constexpr float HARD_IRON_X = 93.76f;
static constexpr float HARD_IRON_Y = -29.09f;
static constexpr float HARD_IRON_Z = 967.01f;

static gl::VirtualGyro vg;
static std::mutex vg_lock;

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
typedef int (*registerSensor_t)(ASensorEventQueue*, ASensor const*, int32_t, int64_t);
typedef int (*enableSensor_t)(ASensorEventQueue*, ASensor const*);
typedef int (*getSensorType_t)(ASensor const*);

static getEvents_t orig_getEvents = nullptr;
static setEventRate_t orig_setEventRate = nullptr;
static registerSensor_t orig_registerSensor = nullptr;
static enableSensor_t orig_enableSensor = nullptr;
static getSensorType_t fn_getSensorType = nullptr;

static const ASensor* g_hw_accel = nullptr;
static std::atomic<ASensorEventQueue*> g_gyro_queue{nullptr};
static std::atomic<bool> g_game_wants_accel_on_gyro_q{false};
static std::atomic<bool> g_hijacked_accel_clock{false};
static std::atomic<int32_t> g_gyro_handle{-1};
static std::atomic<int32_t> g_gyro_type{ASENSOR_TYPE_GYROSCOPE};

static std::atomic<int> own_state{0};
static std::atomic_flag own_started = ATOMIC_FLAG_INIT;

// State for synthetic packet injection & monotonic timestamping
static int64_t g_last_gyro_ts = 0;
static uint64_t g_last_emitted_seq = 0;
static bool g_pending_synth = false;

static inline void feed_accel(const ASensorEvent& e) {
    vg.on_accel({e.acceleration.x, e.acceleration.y, e.acceleration.z}, e.timestamp);
}
static inline void feed_mag_uncal(const ASensorEvent& e) {
    vg.on_mag({e.uncalibrated_magnetic.x_uncalib - HARD_IRON_X,
               e.uncalibrated_magnetic.y_uncalib - HARD_IRON_Y,
               e.uncalibrated_magnetic.z_uncalib - HARD_IRON_Z}, e.timestamp);
}
static inline void feed_mag_cal(const ASensorEvent& e) {
    vg.on_mag({e.magnetic.x, e.magnetic.y, e.magnetic.z}, e.timestamp);
}

static inline void write_synthetic_gyro(ASensorEvent& dst, int32_t handle, int32_t type, float wx, float wy, float wz, int64_t base_ts) {
    memset(&dst, 0, sizeof(ASensorEvent));
    dst.version = sizeof(ASensorEvent);
    dst.sensor = handle;
    dst.type = type;
    dst.gyro.status = ASENSOR_STATUS_ACCURACY_HIGH;

    int64_t latest = vg.latest_ts.load(std::memory_order_relaxed);
    int64_t ts = (latest > base_ts) ? latest : base_ts;
    if (ts <= g_last_gyro_ts) ts = g_last_gyro_ts + 100000; // Strictly monotonic (+0.1ms min step)
    g_last_gyro_ts = ts;
    dst.timestamp = ts;

    if (type == ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED || type == 16) {
        dst.uncalibrated_gyro.x_uncalib = wx;
        dst.uncalibrated_gyro.y_uncalib = wy;
        dst.uncalibrated_gyro.z_uncalib = wz;
        dst.uncalibrated_gyro.x_bias = 0.0f;
        dst.uncalibrated_gyro.y_bias = 0.0f;
        dst.uncalibrated_gyro.z_bias = 0.0f;
    } else {
        dst.gyro.x = wx;
        dst.gyro.y = wy;
        dst.gyro.z = wz;
    }
}

#define VG_LOOPER_ID 3

static void* sensor_thread_main(void*) {
    pthread_setname_np(pthread_self(), "AudioTrack");
    prctl(PR_SET_TIMERSLACK, 1, 0, 0, 0);

    struct sched_param sp = {};
    sp.sched_priority = 2;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        pid_t tid = gettid();
        if (setpriority(PRIO_PROCESS, tid, -20) != 0 && setpriority(PRIO_PROCESS, tid, -16) != 0) {
            setpriority(PRIO_PROCESS, tid, -8);
        }
    }

    void* lib = dlopen("libandroid.so", RTLD_NOW);
    if (!lib) { own_state.store(-1, std::memory_order_relaxed); return nullptr; }

    auto getInstanceForPackage = (ASensorManager* (*)(const char*))dlsym(lib, "ASensorManager_getInstanceForPackage");
    auto getInstance           = (ASensorManager* (*)())dlsym(lib, "ASensorManager_getInstance");
    auto getDefaultSensor      = (const ASensor* (*)(ASensorManager*, int))dlsym(lib, "ASensorManager_getDefaultSensor");
    auto looperPrepare         = (ALooper* (*)(int))dlsym(lib, "ALooper_prepare");
    auto createEventQueue      = (ASensorEventQueue* (*)(ASensorManager*, ALooper*, int, ALooper_callbackFunc, void*))dlsym(lib, "ASensorManager_createEventQueue");
    auto pollOnce              = (int (*)(int, int*, int*, void**))dlsym(lib, "ALooper_pollOnce");
    if (!pollOnce) pollOnce    = (int (*)(int, int*, int*, void**))dlsym(lib, "ALooper_pollAll");

    auto enableSensor   = orig_enableSensor ? orig_enableSensor : (enableSensor_t)dlsym(lib, "ASensorEventQueue_enableSensor");
    auto setEventRate   = orig_setEventRate ? orig_setEventRate : (setEventRate_t)dlsym(lib, "ASensorEventQueue_setEventRate");
    auto registerSensor = orig_registerSensor ? orig_registerSensor : (registerSensor_t)dlsym(lib, "ASensorEventQueue_registerSensor");
    auto getEvents      = orig_getEvents ? orig_getEvents : (getEvents_t)dlsym(lib, "ASensorEventQueue_getEvents");

    if ((!getInstanceForPackage && !getInstance) || !getDefaultSensor || !looperPrepare || !createEventQueue || !enableSensor || !setEventRate || !pollOnce || !getEvents) {
        own_state.store(-1, std::memory_order_relaxed); return nullptr;
    }

    char pkg[256] = {0};
    if (FILE* f = fopen("/proc/self/cmdline", "r")) { size_t n = fread(pkg, 1, sizeof(pkg) - 1, f); pkg[n] = 0; fclose(f); }
    ASensorManager* mgr = (getInstanceForPackage && pkg[0]) ? getInstanceForPackage(pkg) : nullptr;
    if (!mgr && getInstance) mgr = getInstance();
    if (!mgr) { own_state.store(-1, std::memory_order_relaxed); return nullptr; }

    ALooper* looper = looperPrepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    ASensorEventQueue* q = looper ? createEventQueue(mgr, looper, VG_LOOPER_ID, nullptr, nullptr) : nullptr;
    const ASensor* acc = getDefaultSensor(mgr, ASENSOR_TYPE_ACCELEROMETER);
    g_hw_accel = acc; // Save hardware accelerometer pointer for waking the game's gyro queue
    int mag_type = 14;
    const ASensor* mag = getDefaultSensor(mgr, 14);
    if (!mag) { mag_type = 2; mag = getDefaultSensor(mgr, 2); }
    if (!q || !acc || !mag) { own_state.store(-1, std::memory_order_relaxed); return nullptr; }

    if (registerSensor) {
        if (registerSensor(q, acc, 0, 0) < 0 || registerSensor(q, mag, 0, 0) < 0) {
            own_state.store(-1, std::memory_order_relaxed); return nullptr;
        }
    } else if (enableSensor(q, acc) < 0 || enableSensor(q, mag) < 0) {
        own_state.store(-1, std::memory_order_relaxed); return nullptr;
    }
    setEventRate(q, acc, 0);
    setEventRate(q, mag, 0);
    own_state.store(1, std::memory_order_release);

    ASensorEvent ev[64];
    for (;;) {
        int ident = pollOnce(500, nullptr, nullptr, nullptr);
        if (ident == ALOOPER_POLL_ERROR) break;
        if (ident != VG_LOOPER_ID) continue;
        ssize_t n;
        while ((n = getEvents(q, ev, 64)) > 0) {
            for (ssize_t i = 0; i < n; i++) {
                if (ev[i].type == ASENSOR_TYPE_ACCELEROMETER) { feed_accel(ev[i]); }
                else if (mag_type == 14 && ev[i].type == 14)  { feed_mag_uncal(ev[i]); }
                else if (mag_type == 2 && ev[i].type == 2)    { feed_mag_cal(ev[i]); }
            }
        }
    }
    own_state.store(-1, std::memory_order_release);
    return nullptr;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    if (__builtin_expect(!own_started.test_and_set(std::memory_order_relaxed), 0)) {
        pthread_t th;
        if (pthread_create(&th, nullptr, sensor_thread_main, nullptr) == 0) pthread_detach(th);
        else own_state.store(-1, std::memory_order_relaxed);
    }

    // 1-Shot synthetic packet delivery when the previous call had count == 1 and couldn't append
    if (__builtin_expect(g_pending_synth && queue == g_gyro_queue.load(std::memory_order_relaxed) && count >= 1, 0)) {
        g_pending_synth = false;
        int32_t handle = g_gyro_handle.load(std::memory_order_relaxed);
        if (handle != -1) {
            const float wx = vg.out_x.load(std::memory_order_relaxed);
            const float wy = vg.out_y.load(std::memory_order_relaxed);
            const float wz = vg.out_z.load(std::memory_order_relaxed);
            write_synthetic_gyro(events[0], handle, g_gyro_type.load(std::memory_order_relaxed), wx, wy, wz, vg.latest_ts.load(std::memory_order_relaxed));
            return 1;
        }
    }

    ssize_t n = orig_getEvents(queue, events, count);
    if (n <= 0) return n;

    const int state = own_state.load(std::memory_order_acquire);
    if (__builtin_expect(state == 1, 1)) {
        const float wx = vg.out_x.load(std::memory_order_relaxed);
        const float wy = vg.out_y.load(std::memory_order_relaxed);
        const float wz = vg.out_z.load(std::memory_order_relaxed);
        const bool is_gyro_q = (queue == g_gyro_queue.load(std::memory_order_relaxed));
        const bool game_wants_acc = g_game_wants_accel_on_gyro_q.load(std::memory_order_relaxed);

        bool emitted_gyro_in_batch = false;
        int64_t batch_ts = 0;

        for (ssize_t i = 0; i < n; i++) {
            ASensorEvent& e = events[i];
            if (e.timestamp > batch_ts) batch_ts = e.timestamp;

            if (e.type == ASENSOR_TYPE_GYROSCOPE || e.type == 4 || e.type == ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED || e.type == 16) {
                // Capture handle & queue on first gyro packet, then hijack hardware Accel to clock this queue at 200Hz+
                if (__builtin_expect(g_gyro_handle.load(std::memory_order_relaxed) == -1, 0)) {
                    g_gyro_handle.store(e.sensor, std::memory_order_relaxed);
                    g_gyro_type.store(e.type, std::memory_order_relaxed);
                    g_gyro_queue.store(queue, std::memory_order_relaxed);
                }
                if (__builtin_expect(!g_hijacked_accel_clock.load(std::memory_order_relaxed) && g_hw_accel != nullptr, 0)) {
                    g_hijacked_accel_clock.store(true, std::memory_order_relaxed);
                    if (orig_registerSensor) orig_registerSensor(queue, g_hw_accel, 0, 0);
                    else if (orig_enableSensor) orig_enableSensor(queue, g_hw_accel);
                    if (orig_setEventRate) orig_setEventRate(queue, g_hw_accel, 0);
                }

                write_synthetic_gyro(e, e.sensor, e.type, wx, wy, wz, e.timestamp);
                emitted_gyro_in_batch = true;
                g_last_emitted_seq = vg.seq.load(std::memory_order_relaxed);
            } else if (e.type == ASENSOR_TYPE_ACCELEROMETER && is_gyro_q && !game_wants_acc) {
                // Dedicated gyro queue woken up by our hijacked 200Hz+ hardware Accel: transmute in-place into a 0-delay Gyro packet!
                int32_t handle = g_gyro_handle.load(std::memory_order_relaxed);
                if (handle != -1) {
                    write_synthetic_gyro(e, handle, g_gyro_type.load(std::memory_order_relaxed), wx, wy, wz, e.timestamp);
                    emitted_gyro_in_batch = true;
                    g_last_emitted_seq = vg.seq.load(std::memory_order_relaxed);
                }
            }
        }

        // Shared queue path: if woken by Accel and no Gyro packet was in this batch, inject a synthetic Gyro packet
        uint64_t cur_seq = vg.seq.load(std::memory_order_acquire);
        if (is_gyro_q && !emitted_gyro_in_batch && cur_seq != g_last_emitted_seq) {
            int32_t handle = g_gyro_handle.load(std::memory_order_relaxed);
            if (handle != -1) {
                g_last_emitted_seq = cur_seq;
                if ((size_t)n < count) {
                    write_synthetic_gyro(events[n], handle, g_gyro_type.load(std::memory_order_relaxed), wx, wy, wz, batch_ts);
                    n++;
                } else {
                    g_pending_synth = true; // Will be emitted on the immediate next iteration of the game's while() loop
                }
            }
        }
        return n;
    }

    // Fallback path only if private sensor thread is not active
    std::lock_guard<std::mutex> lock(vg_lock);
    for (ssize_t i = 0; i < n; i++) {
        ASensorEvent& e = events[i];
        if (e.type == ASENSOR_TYPE_ACCELEROMETER) {
            feed_accel(e);
        } else if (e.type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || e.type == 14) {
            feed_mag_uncal(e);
        } else if (e.type == ASENSOR_TYPE_GYROSCOPE || e.type == 4 || e.type == ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED || e.type == 16) {
            const float wx = vg.out_x.load(std::memory_order_relaxed);
            const float wy = vg.out_y.load(std::memory_order_relaxed);
            const float wz = vg.out_z.load(std::memory_order_relaxed);
            write_synthetic_gyro(e, e.sensor, e.type, wx, wy, wz, e.timestamp);
        }
    }
    return n;
}

static void track_queue_subscription(ASensorEventQueue* queue, ASensor const* sensor) {
    if (!fn_getSensorType || !sensor) return;
    int type = fn_getSensorType(sensor);
    if (type == ASENSOR_TYPE_GYROSCOPE || type == 4 || type == ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED || type == 16) {
        g_gyro_queue.store(queue, std::memory_order_relaxed);
    } else if (type == ASENSOR_TYPE_ACCELEROMETER) {
        if (queue == g_gyro_queue.load(std::memory_order_relaxed)) {
            g_game_wants_accel_on_gyro_q.store(true, std::memory_order_relaxed);
        }
    }
}

int hook_ASensorEventQueue_setEventRate(ASensorEventQueue* queue, ASensor const* sensor, int32_t) {
    track_queue_subscription(queue, sensor);
    return orig_setEventRate(queue, sensor, 0);
}

int hook_ASensorEventQueue_registerSensor(ASensorEventQueue* queue, ASensor const* sensor, int32_t, int64_t) {
    track_queue_subscription(queue, sensor);
    return orig_registerSensor(queue, sensor, 0, 0);
}

int hook_ASensorEventQueue_enableSensor(ASensorEventQueue* queue, ASensor const* sensor) {
    track_queue_subscription(queue, sensor);
    int res = orig_enableSensor(queue, sensor);
    if (res >= 0 && orig_setEventRate) orig_setEventRate(queue, sensor, 0);
    return res;
}

void install_hook() {
    void* libandroid = dlopen("libandroid.so", RTLD_NOW);
    if (!libandroid) return;
    fn_getSensorType  = (getSensorType_t)dlsym(libandroid, "ASensor_getType");
    void* target_get  = dlsym(libandroid, "ASensorEventQueue_getEvents");
    void* target_rate = dlsym(libandroid, "ASensorEventQueue_setEventRate");
    void* target_reg  = dlsym(libandroid, "ASensorEventQueue_registerSensor");
    void* target_en   = dlsym(libandroid, "ASensorEventQueue_enableSensor");

    if (target_get)  DobbyHook(target_get,  (dobby_dummy_func_t)hook_ASensorEventQueue_getEvents,      (dobby_dummy_func_t*)&orig_getEvents);
    if (target_rate) DobbyHook(target_rate, (dobby_dummy_func_t)hook_ASensorEventQueue_setEventRate,   (dobby_dummy_func_t*)&orig_setEventRate);
    if (target_reg)  DobbyHook(target_reg,  (dobby_dummy_func_t)hook_ASensorEventQueue_registerSensor, (dobby_dummy_func_t*)&orig_registerSensor);
    if (target_en)   DobbyHook(target_en,   (dobby_dummy_func_t)hook_ASensorEventQueue_enableSensor,   (dobby_dummy_func_t*)&orig_enableSensor);
}

class GyroModifier : public zygisk::ModuleBase {
public:
    void onLoad(zygisk::Api* api, JNIEnv* env) override { this->api = api; this->env = env; }
    void preAppSpecialize(zygisk::AppSpecializeArgs* args) override {
        const char* process = env->GetStringUTFChars(args->nice_name, nullptr);
        if (!process) return;
        static const char* const targets[] = {
            "com.activision.callofduty.shooter",
            "com.garena.game.codm",
            "com.tencent.tmgp.cod",
            "com.tencent.ig",
            "com.pubg.imobile"
        };
        for (const char* t : targets) {
            if (strcmp(process, t) == 0) { enable_hack = true; break; }
        }
        env->ReleaseStringUTFChars(args->nice_name, process);
    }
    void postAppSpecialize(const zygisk::AppSpecializeArgs*) override {
        if (enable_hack) install_hook();
    }
private:
    zygisk::Api* api = nullptr;
    JNIEnv* env = nullptr;
    bool enable_hack = false;
};
REGISTER_ZYGISK_MODULE(GyroModifier)