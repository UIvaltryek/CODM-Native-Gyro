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
// Virtual gyroscope (accelerometer + magnetometer) - Kalman-style tracker.
// Multithreaded Private Queue + Pure Quaternion Kinematics.
// Optimized for Call of Duty: Mobile / BGMI.
// =====================================================================================================

namespace gl {

typedef double real;
struct D3 { real x, y, z; };                                         
struct V3 { float x, y, z; };                                       
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
static inline Quat qconj(Quat a) { return {a.w, -a.x, -a.y, -a.z}; }
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
static inline D3 qlog(Quat q) {                                     
    if (q.w < 0.0) q = {-q.w, -q.x, -q.y, -q.z};
    D3 v = {q.x, q.y, q.z}; real s = len(v);
    if (s < 1e-12) return v * 2.0;
    return v * (2.0 * atan2(s, q.w) / s);
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
    real sigma_tilt  = 0.01;     
    real sigma_head  = 0.01;     
    real hand_acc    = 150.0;     
    real g_tol       = 35.0;      
    real mag_tol     = 1.00;     
    real dip_tol     = 3.13962634015954636; 
    real jump_reject = 3.14159; 
    real tilt_reject = 3.14159;     
    real outage_tau  = 0.15;     
    real deadzone    = 0.0;      
    real lock_hz     = 0.0;      
    real max_gap     = 1.0;      
    real mag_wait    = 0.5;      
};

class VirtualGyro {
public:
    explicit VirtualGyro(Params p = Params()) : P(p) { reset(); }

    void reset() {
        q = q_out = {1.0, 0.0, 0.0, 0.0}; w = last_out = a0 = m0 = {0.0, 0.0, 0.0};
        inited = have_a = have_m = ref_valid = acc_seen = mag_seen = false;
        t_last = t_out = t_acc = t_mag = ta0 = tm0 = ta_first = 0;
        m_ref = dip_ref = 0.0; h_refx = 0.0; h_refy = 1.0; reject_t = reject_tilt_t = 0.0; wa = wm = 0.0;
    }

    void onAccel(V3 a, int64_t t) { on_accel({a.x, a.y, a.z}, t); }
    void onMag(V3 m, int64_t t)   { on_mag({m.x, m.y, m.z}, t); }
    V3 sample(int64_t t) { D3 r = sample_d(t); return {(float)r.x, (float)r.y, (float)r.z}; }

    void on_accel(D3 a, int64_t t) {
        real n = len(a); if (!(n > 1e-3)) return;                     
        if (inited) advance(t);                                       
        if (!inited) { a0 = a; ta0 = t; if (!have_a) { have_a = true; ta_first = t; } try_init(t); return; }
        real T = interval(t, t_acc, acc_seen); t_acc = t; acc_seen = true;
        D3 up = up_in_dev(q);
        D3 e = rotvec_between(a * (1.0 / n), up);                     
        if (len(e) > P.tilt_reject) {                                 
            reject_tilt_t += T; wa = 0.0;
            if (reject_tilt_t > 0.3) { restart(); a0 = a; ta0 = t; have_a = true; ta_first = t; return; }
            decay(up, T, false); return;
        }
        reject_tilt_t = 0.0;
        wa = soft_gate(fabs(n - G), P.g_tol);                         
        if (wa < 0.02) { decay(up, T, false); return; }               
        correct(e, T, P.sigma_tilt / sqrt(wa));
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
        wm = wgt;
        if (wgt < 0.02) {
            reject_t += T;
            if (reject_t > 0.5 && hn >= 0.2 * nm) {                   
                h_refx = hx / hn; h_refy = hy / hn; m_ref = nm; dip_ref = dip; reject_t = 0.0;
            }
            decay(up, T, true);                                       
            return;
        }
        reject_t = 0.0;
        correct(up * psi, T, P.sigma_head / sqrt(wgt));               
        if (wgt > 0.9) {                                              
            real k = fmin(1.0, T / 20.0);
            m_ref += k * (nm - m_ref); dip_ref += k * (dip - dip_ref);
        }
    }

    D3 sample_d(int64_t t) {
        if (inited) advance(t);
        if (!inited) return {0.0, 0.0, 0.0};
        if (t <= t_out) return last_out;
        real dt = (t - t_out) * 1e-9;
        D3 wo = w; real n = len(w);
        wo = (n <= P.deadzone) ? D3{0.0, 0.0, 0.0} : w * (1.0 - P.deadzone / n);
        if (dt > P.max_gap) { q_out = q; t_out = t; last_out = wo; return wo; }
        if (P.lock_hz > 0.0)                                          
            wo = wo + qlog(qmul(qconj(q_out), q)) * (6.283185307179586 * P.lock_hz);
        q_out = qnorm(qmul(q_out, qexp(wo * dt)));
        t_out = t; last_out = wo;
        return wo;
    }

    bool  ready()       const { return inited; }                  
    bool  has_heading() const { return inited && ref_valid; }     
    float rate_now()    const { return (float)len(last_out); }    
    float tilt_trust() const { return (float)wa; }
    float yaw_trust()  const { return (float)wm; }
    Quat  attitude()   const { return q; }

private:
    static constexpr real G = 9.80665;
    Params P;
    Quat q, q_out; D3 w, last_out, a0, m0;
    bool inited, have_a, have_m, ref_valid, acc_seen, mag_seen;
    int64_t t_last, t_out, t_acc, t_mag, ta0, tm0, ta_first;
    real m_ref, dip_ref, h_refx, h_refy, reject_t, reject_tilt_t, wa, wm;

    static real interval(int64_t t, int64_t prev, bool seen) {
        if (!seen) return 0.01;
        real T = (t - prev) * 1e-9; return fmin(0.1, fmax(1e-3, T));
    }

    void correct(D3 e, real T, real sigma_meas) {
        real lam = P.hand_acc * T * T / sigma_meas;                   
        real r = (4.0 + lam - sqrt(8.0 * lam + lam * lam)) / 4.0;
        real alpha = fmin(1.0 - r * r, 0.95);
        real beta = fmin(fmax(2.0 * (2.0 - (1.0 - r * r)) - 4.0 * r, 0.0), 0.85);
        q = qnorm(qmul(q, qexp(e * alpha)));
        w = w + e * (beta / T);
    }

    void decay(D3 g, real T, bool along) {                            
        real k = exp(-T / P.outage_tau);
        D3 par = g * dot(w, g);
        w = along ? (par * k + (w - par)) : (par + (w - par) * k);
    }

    void advance(int64_t t) {
        if (t <= t_last) return;
        real dt = (t - t_last) * 1e-9;
        if (dt > P.max_gap) { restart(); return; }
        q = qnorm(qmul(q, qexp(w * dt)));
        t_last = t;
    }

    void restart() {
        inited = have_a = have_m = ref_valid = acc_seen = mag_seen = false;
        w = {0.0, 0.0, 0.0}; reject_t = reject_tilt_t = 0.0;
    }

    void start(int64_t t) {
        inited = true; w = last_out = {0.0, 0.0, 0.0}; q_out = q; t_last = t_out = t;
        acc_seen = mag_seen = false; reject_t = reject_tilt_t = 0.0;
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
// HOOKS & MULTITHREADING - Sensor polling + NDK overrides
// =====================================================================================================

static const float HARD_IRON_X = 93.76f;      
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

static gl::VirtualGyro vg;                    
static std::mutex vg_lock;                    

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static getEvents_t orig_getEvents = nullptr;
static setEventRate_t orig_setEventRate = nullptr;

static std::atomic<int> own_state{0};                                                       

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
    a.getEvents   = orig_getEvents ? orig_getEvents : (decltype(a.getEvents))dlsym(lib, "ASensorEventQueue_getEvents");
    a.setEventRate = orig_setEventRate ? orig_setEventRate : (decltype(a.setEventRate))dlsym(lib, "ASensorEventQueue_setEventRate");
    return (a.getInstanceForPackage || a.getInstance) && a.getDefaultSensor && a.looperPrepare && a.createEventQueue &&
           a.enableSensor && a.setEventRate && a.pollOnce && a.getEvents;
}
static bool (*ndk_loader)(NdkApi&) = load_ndk;       

#define VG_LOOPER_ID 3                               

static void* sensor_thread_main(void* arg) {
    NdkApi api = *(NdkApi*)arg; delete (NdkApi*)arg;
    char pkg[256] = {0};
    if (FILE* f = fopen("/proc/self/cmdline", "r")) { size_t n = fread(pkg, 1, sizeof(pkg) - 1, f); pkg[n] = 0; fclose(f); }
    ASensorManager* mgr = (api.getInstanceForPackage && pkg[0]) ? api.getInstanceForPackage(pkg) : nullptr;
    if (!mgr && api.getInstance) mgr = api.getInstance();
    if (!mgr) { own_state.store(-1); return nullptr; }
    ALooper* looper = api.looperPrepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    ASensorEventQueue* q = looper ? api.createEventQueue(mgr, looper, VG_LOOPER_ID, nullptr, nullptr) : nullptr;
    const ASensor* acc = api.getDefaultSensor(mgr, ASENSOR_TYPE_ACCELEROMETER);
    int mag_type = 14;
    const ASensor* mag = api.getDefaultSensor(mgr, 14);                  
    if (!mag) { mag_type = 2; mag = api.getDefaultSensor(mgr, 2); }      
    if (!q || !acc || !mag) { own_state.store(-1); return nullptr; }
    if (api.enableSensor(q, acc) < 0 || api.enableSensor(q, mag) < 0) { own_state.store(-1); return nullptr; }
    
    // Max hardware pull: 200 Hz Accel / 200 Hz Mag (or as fast as the chip physically supports)
    api.setEventRate(q, acc, 5000);                                      
    api.setEventRate(q, mag, 5000);                                     
    own_state.store(1);
    
    ASensorEvent ev[64];
    for (;;) {
        int ident = api.pollOnce(500, nullptr, nullptr, nullptr);
        if (ident == ALOOPER_POLL_ERROR) break;
        if (ident != VG_LOOPER_ID) continue;
        ssize_t n;
        while ((n = api.getEvents(q, ev, 64)) > 0) {
            std::lock_guard<std::mutex> lock(vg_lock);
            for (ssize_t i = 0; i < n; i++) {
                if (ev[i].type == ASENSOR_TYPE_ACCELEROMETER) { feed_accel(ev[i]); }
                else if (mag_type == 14 && ev[i].type == 14)  { feed_mag_uncal(ev[i]); }
                else if (mag_type == 2 && ev[i].type == 2)    { feed_mag_cal(ev[i]); }
            }
        }
    }
    own_state.store(-1);
    return nullptr;
}

static std::atomic_flag own_started = ATOMIC_FLAG_INIT;
static void start_sensor_thread() {
    NdkApi* api = new NdkApi();
    if (!ndk_loader(*api)) { own_state.store(-1); delete api; return; }
    pthread_t th;
    if (pthread_create(&th, nullptr, sensor_thread_main, api) == 0) pthread_detach(th);
    else { own_state.store(-1); delete api; }
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t n = orig_getEvents(queue, events, count);
    if (!own_started.test_and_set()) { start_sensor_thread(); }
    if (n <= 0) return n;
    const bool own = own_state.load() == 1;        
    std::lock_guard<std::mutex> lock(vg_lock);
    for (ssize_t i = 0; i < n; i++) {
        ASensorEvent& e = events[i];
        if (e.type == ASENSOR_TYPE_ACCELEROMETER) {
            if (!own) feed_accel(e);
        } else if (e.type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || e.type == 14) {
            if (!own) feed_mag_uncal(e);
        } else if (e.type == ASENSOR_TYPE_GYROSCOPE || e.type == 4) {
            gl::V3 w = vg.sample(e.timestamp);          
            e.gyro.x = w.x; e.gyro.y = w.y; e.gyro.z = w.z;
        } else if (e.type == ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED || e.type == 16) {
            gl::V3 w = vg.sample(e.timestamp);          
            e.uncalibrated_gyro.x_uncalib = w.x; 
            e.uncalibrated_gyro.y_uncalib = w.y; 
            e.uncalibrated_gyro.z_uncalib = w.z;
            e.uncalibrated_gyro.x_bias = 0.0f;
            e.uncalibrated_gyro.y_bias = 0.0f;
            e.uncalibrated_gyro.z_bias = 0.0f;
        }
    }
    return n;
}

int hook_ASensorEventQueue_setEventRate(ASensorEventQueue* queue, ASensor const* sensor, int32_t usec) {
    // Overrides all game delay requests and forces ASENSOR_DELAY_FASTEST (0)
    return orig_setEventRate(queue, sensor, 0);      
}

void install_hook() {
    void* libandroid = dlopen("libandroid.so", RTLD_NOW);
    if (libandroid) {
        void* target_get = dlsym(libandroid, "ASensorEventQueue_getEvents");
        void* target_rate = dlsym(libandroid, "ASensorEventQueue_setEventRate");
        if (target_get) DobbyHook(target_get, (dobby_dummy_func_t)hook_ASensorEventQueue_getEvents, (dobby_dummy_func_t*)&orig_getEvents);
        if (target_rate) DobbyHook(target_rate, (dobby_dummy_func_t)hook_ASensorEventQueue_setEventRate, (dobby_dummy_func_t*)&orig_setEventRate);
    }
}

class GyroModifier : public zygisk::ModuleBase {
public:
    void onLoad(zygisk::Api* api, JNIEnv* env) override { this->api = api; this->env = env; }
    void preAppSpecialize(zygisk::AppSpecializeArgs* args) override {
        const char* process = env->GetStringUTFChars(args->nice_name, nullptr);
        if (process) {
            static const char* const targets[] = { "com.activision.callofduty.shooter",   
                                                   "com.garena.game.codm",                 
                                                   "com.tencent.tmgp.cod",                 
                                                   "com.tencent.ig",                       
                                                   "com.pubg.imobile" };                   
            for (const char* t : targets) if (strcmp(process, t) == 0) enable_hack = true;
            env->ReleaseStringUTFChars(args->nice_name, process);
        }
    }
    void postAppSpecialize(const zygisk::AppSpecializeArgs*) override { if (enable_hack) install_hook(); }
private:
    zygisk::Api* api = nullptr; JNIEnv* env = nullptr; bool enable_hack = false;
};
REGISTER_ZYGISK_MODULE(GyroModifier)
