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
#include <cmath>
#include <atomic>
#include <mutex>
#include "dobby.h"
#include "zygisk.hpp"

namespace vg {

using Real = double;
constexpr Real kPi = 3.14159265358979323846;

struct Vec3 { Real x = 0, y = 0, z = 0; };
inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(const Vec3& a, Real s)        { return {a.x * s, a.y * s, a.z * s}; }
inline Real dot(const Vec3& a, const Vec3& b)       { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline Real norm(const Vec3& a) { return std::sqrt(dot(a, a)); }

struct Quat { Real w = 1, x = 0, y = 0, z = 0; };
inline Quat operator*(const Quat& a, const Quat& b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
inline Quat conj(const Quat& q) { return {q.w, -q.x, -q.y, -q.z}; }
inline Quat normalized(const Quat& q) {
    const Real n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!(n > 1e-12)) return Quat{};
    const Real k = 1.0 / n;
    return {q.w * k, q.x * k, q.y * k, q.z * k};
}
inline Vec3 rotate(const Quat& q, const Vec3& v) {
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 t = cross(u, v) * 2.0;
    return v + t * q.w + cross(u, t);
}
inline Vec3 rotateInv(const Quat& q, const Vec3& v) { return rotate(conj(q), v); }

inline Quat expMap(const Vec3& phi) {
    const Real a2 = dot(phi, phi);
    Real s, c;
    if (a2 < 1e-10) { s = 0.5 - a2 / 48.0; c = 1.0 - a2 / 8.0; }
    else { const Real a = std::sqrt(a2); s = std::sin(0.5 * a) / a; c = std::cos(0.5 * a); }
    return {c, phi.x * s, phi.y * s, phi.z * s};
}
inline Vec3 logMap(Quat q) {
    if (q.w < 0) q = {-q.w, -q.x, -q.y, -q.z};
    const Real s = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    const Real k = (s < 1e-9) ? 2.0 / q.w : 2.0 * std::atan2(s, q.w) / s;
    return {q.x * k, q.y * k, q.z * k};
}
inline Vec3 arcVector(const Vec3& a, const Vec3& b) {
    const Vec3 c = cross(a, b);
    const Real s = norm(c), d = dot(a, b);
    if (s > 1e-12) return c * (std::atan2(s, d) / s);
    if (d > 0) return Vec3{};
    const Real ax = std::fabs(a.x), ay = std::fabs(a.y), az = std::fabs(a.z);
    const Vec3 e = (ax <= ay && ax <= az) ? Vec3{1, 0, 0} : (ay <= az ? Vec3{0, 1, 0} : Vec3{0, 0, 1});
    const Vec3 p = cross(a, e);
    return p * (kPi / norm(p));
}
inline Real clamp01(Real x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }
inline Real smoothstep(Real e0, Real e1, Real x) {
    const Real t = clamp01((x - e0) / (e1 - e0));
    return t * t * (3 - 2 * t);
}

struct Config {
    Real gravity       = 9.80665;
    // Raised noise floor so oscillating speaker/DAC vibrations stay below the EMA activity threshold
    Real accelNoise    = 0.08;
    Real magNoise      = 1.2;

    // Tilt & Yaw bandwidths: ultra-calm at rest (no shooting jitter), wide-open on movement (~6-10ms response)
    Real bwRest        = 5.0;
    Real bwMax         = 160.0;
    Real bwPerSpeed    = 85.0;
    Real bwRestYaw     = 4.0;
    Real bwMaxYaw      = 140.0;   // Matched to tilt so horizontal flicks never lag behind vertical

    // Output follower: 1:1 angle lock without deadzone
    Real outBwRest     = 5.0;
    Real outBwPerSpeed = 500.0;
    Real restDeadband  = 0.0;     // Zero deadzone for micro-aiming

    // Wide-open gates so high-G centripetal flicks and violent swipes are never rejected
    Real accGateLo     = 3.5;
    Real accGateHi     = 8.0;
    Real outlierSigma  = 200.0;
    Real maxAngAccel   = 25000.0;
    Real innovLambda   = 0.35;
    Real decayTau      = 0.15;
    Real resyncAfter   = 0.30;
    Real maxGap        = 1.0;
    Real magStale      = 0.25;
    Real magRefTau     = 4.0;
    int  magWarmup     = 8;
    Real magReanchor   = 1.5;
};

class VirtualGyro {
public:
    std::atomic<float> out_x{0.0f}, out_y{0.0f}, out_z{0.0f};
    std::atomic<int64_t> latest_ts{0};
    std::atomic<uint64_t> seq{0};

    explicit VirtualGyro(const Config& cfg = Config()) : cfg_(cfg) { reset(); }

    void reset() {
        init_ = haveMagRef_ = havePending_ = false;
        q_ = y_ = Quat{};
        w_ = out_ = rhoAvgT_ = magPending_ = mwAvg_ = mwSum_ = Vec3{};
        northRef_ = Vec3{0, 1, 0}; magWarm_ = 0;
        tState_ = tAcc_ = tMag_ = 0;
        actT_ = actY_ = psiAvg_ = refB_ = refDip_ = magBad_ = rejectTime_ = 0;
        trustT_ = 1; trustY_ = 0;
        gRef_ = cfg_.gravity;
        out_x.store(0.0f, std::memory_order_relaxed);
        out_y.store(0.0f, std::memory_order_relaxed);
        out_z.store(0.0f, std::memory_order_relaxed);
    }

    void onMag(int64_t tNs, Real mx, Real my, Real mz) {
        const Vec3 m{mx, my, mz};
        const Real mn = norm(m);
        if (!std::isfinite(mn) || mn < 1.0) return;
        if (!init_) { magPending_ = m; havePending_ = true; return; }
        if (!haveMagRef_) { anchorMag(m); tMag_ = tNs; return; }

        const Real T = (tNs - tMag_) * 1e-9;
        if (T <= 0) return;
        tMag_ = tNs;
        if (T > cfg_.maxGap) return;
        propagateTo(tNs);

        const Vec3 up = rotateInv(q_, Vec3{0, 0, 1});
        const Real mUp = dot(m, up);
        const Vec3 mh = m - up * mUp;
        const Real mhn = norm(mh);
        const Real cd = mUp / mn;

        if (magWarm_ < cfg_.magWarmup) {
            mwSum_ = mwSum_ + rotate(q_, m);
            ++magWarm_;
            refreshNorthRef(mwSum_ * (1.0 / magWarm_), mn);
            refB_ += (mn - refB_) / magWarm_;
            refDip_ += (cd - refDip_) / magWarm_;
            return;
        }

        // Expanded field gates so cross-axis bleed during wide flicks never blinds yaw
        const Real eB = std::fabs(mn / refB_ - 1.0), eD = std::fabs(cd - refDip_);
        const Real gate = (1.0 - smoothstep(2.0, 10.0, eB)) * (1.0 - smoothstep(1.5, 3.14, eD)) *
                          smoothstep(0.05, 0.20, mhn / refB_);

        const Vec3 mw = rotate(q_, m);
        const bool steady = norm(mw - mwAvg_) < 0.05 * refB_;
        mwAvg_ = mwAvg_ + (mw - mwAvg_) * (1.0 - std::exp(-T / 0.4));
        if (gate > 0.9) { magBad_ = 0; }
        else if (gate < 0.3 && steady) {
            magBad_ += T;
            if (magBad_ > cfg_.magReanchor) { anchorMag(m); magBad_ = 0; return; }
        }

        if (mhn < 1e-6) return;
        const Vec3 north = rotateInv(q_, northRef_);
        const Vec3 hm = mh * (1.0 / mhn);
        const Real psi = std::atan2(dot(up, cross(hm, north)), dot(north, hm));

        const Real sigma = 1.25 * cfg_.magNoise / mhn;
        const Real scale = cfg_.outlierSigma * sigma + cfg_.maxAngAccel * T * T;
        const Real sur = std::fabs(psi - psiAvg_) / scale;
        const Real wRob = 1.0 / (1.0 + sur * sur);
        const Real psiW = psi * wRob;
        psiAvg_ += (psiW - psiAvg_) * cfg_.innovLambda;
        const Real fEma = 3.0 * sigma * std::sqrt(cfg_.innovLambda / (2.0 - cfg_.innovLambda));
        actY_ = std::fmax(std::fmax(0.0, std::fabs(psiAvg_) - fEma),
                          std::fmax(0.0, std::fabs(psiW) - 3.5 * sigma)) / T;

        // Decoupled from trustT_ so high-G wrist flicks never blind horizontal turning
        const Real trust = gate * wRob;
        trustY_ += (trust - trustY_) * 0.5;
        if (gate > 0.9) {
            const Real k = 1.0 - std::exp(-T / cfg_.magRefTau);
            refB_ += (mn - refB_) * k;
            refDip_ += (cd - refDip_) * k;
        }
        const Real speed = norm(w_) + actT_ + actY_;
        const Real bw = std::fmin(cfg_.bwMaxYaw, cfg_.bwRestYaw + cfg_.bwPerSpeed * speed);
        correct(up * psi, T, trust, bw);
    }

    void onAccel(int64_t tNs, Real ax, Real ay, Real az) {
        const Vec3 a{ax, ay, az};
        const Real an = norm(a);
        if (!std::isfinite(an) || an < 0.1 * gRef_) return;
        const Vec3 u = a * (1.0 / an);

        if (!init_) { start(tNs, u); return; }
        const Real T = (tNs - tAcc_) * 1e-9;
        if (T <= 0) return;
        if (T > cfg_.maxGap) { reseat(tNs, u); return; }

        propagateTo(tNs);
        const Vec3 up0 = rotateInv(q_, Vec3{0, 0, 1});
        const Vec3 rho = arcVector(u, up0);

        const Real eMag = std::fabs(an - gRef_) / gRef_;
        const Real gMag = 1.0 - smoothstep(cfg_.accGateLo, cfg_.accGateHi, eMag);
        const Real sigma = 1.25 * cfg_.accelNoise / an;
        const Real scale = cfg_.outlierSigma * sigma + cfg_.maxAngAccel * T * T;
        const Real sur = norm(rho - rhoAvgT_) / scale;
        const Real wRob = 1.0 / (1.0 + sur * sur);
        const Real trust = gMag * wRob;

        rejectTime_ = (wRob < 0.2 && gMag > 0.9) ? rejectTime_ + T : 0.0;
        if (rejectTime_ > cfg_.resyncAfter) { reseat(tNs, u); return; }

        const Vec3 rhoW = rho * wRob;
        rhoAvgT_ = rhoAvgT_ + (rhoW - rhoAvgT_) * cfg_.innovLambda;
        const Real fEma = 3.0 * sigma * std::sqrt(cfg_.innovLambda / (2.0 - cfg_.innovLambda));
        actT_ = std::fmax(std::fmax(0.0, norm(rhoAvgT_) - fEma),
                          std::fmax(0.0, norm(rhoW) - 4.0 * sigma)) / T;

        const Real speed = norm(w_) + actT_ + actY_;
        const Real bw = std::fmin(cfg_.bwMax, cfg_.bwRest + cfg_.bwPerSpeed * speed);
        correct(rho, T, trust, bw);

        trustT_ += (trust - trustT_) * (1.0 - std::exp(-T / 0.03));
        if (eMag < 0.05 && norm(w_) < 0.2)
            gRef_ += (an - gRef_) * (1.0 - std::exp(-T / 3.0));

        const Real yawFresh = ((tNs - tMag_) * 1e-9 < cfg_.magStale) ? trustY_ : 0.0;
        decay(rotateInv(q_, Vec3{0, 0, 1}), trustT_, yawFresh, T);

        tAcc_ = tNs;
        Vec3 w_out = emit(T);
        publish(w_out, tNs);
    }

private:
    Config cfg_;
    bool init_, haveMagRef_, havePending_;
    Quat q_, y_;
    Vec3 w_, out_, rhoAvgT_, magPending_, mwAvg_, mwSum_, northRef_;
    int magWarm_ = 0;
    int64_t tState_, tAcc_, tMag_;
    Real actT_, actY_, psiAvg_, refB_, refDip_, magBad_, rejectTime_, trustT_, trustY_, gRef_;

    inline void publish(const Vec3& w_out, int64_t t) {
        out_x.store((float)w_out.x, std::memory_order_relaxed);
        out_y.store((float)w_out.y, std::memory_order_relaxed);
        out_z.store((float)w_out.z, std::memory_order_relaxed);
        if (t > latest_ts.load(std::memory_order_relaxed)) {
            latest_ts.store(t, std::memory_order_relaxed);
        }
        seq.fetch_add(1, std::memory_order_release);
    }

    void start(int64_t tNs, const Vec3& u) {
        q_ = normalized(expMap(arcVector(u, Vec3{0, 0, 1})));
        y_ = q_; w_ = out_ = Vec3{};
        tState_ = tAcc_ = tNs; init_ = true;
        if (havePending_) { anchorMag(magPending_); tMag_ = tNs; havePending_ = false; }
        publish(out_, tNs);
    }
    void reseat(int64_t tNs, const Vec3& u) {
        q_ = normalized(q_ * expMap(arcVector(u, rotateInv(q_, Vec3{0, 0, 1}))));
        y_ = q_; w_ = out_ = rhoAvgT_ = Vec3{};
        actT_ = actY_ = psiAvg_ = rejectTime_ = 0;
        tState_ = tAcc_ = tNs;
        publish(out_, tNs);
    }
    void refreshNorthRef(const Vec3& mWorld, Real mn) {
        const Vec3 h{mWorld.x, mWorld.y, 0.0};
        const Real hn = norm(h);
        if (hn > 0.1 * mn) northRef_ = h * (1.0 / hn);
        mwAvg_ = mWorld;
    }
    void anchorMag(const Vec3& m) {
        const Vec3 up = rotateInv(q_, Vec3{0, 0, 1});
        const Real mn = norm(m);
        refB_ = mn; refDip_ = dot(m, up) / mn;
        mwSum_ = rotate(q_, m); magWarm_ = 1;
        refreshNorthRef(mwSum_, mn);
        psiAvg_ = 0; magBad_ = 0; haveMagRef_ = true;
    }
    void propagateTo(int64_t tNs) {
        const Real dt = (tNs - tState_) * 1e-9;
        if (dt <= 0) return;
        q_ = normalized(q_ * expMap(w_ * dt));
        tState_ = tNs;
    }
    void correct(const Vec3& rho, Real T, Real trust, Real bw) {
        const Real th = std::exp(-bw * T);
        const Real a = (1.0 - th * th) * trust;
        const Real b = (1.0 - th) * (1.0 - th) * trust;
        q_ = normalized(q_ * expMap(rho * a));
        w_ = w_ + rho * (b / T);
    }
    void decay(const Vec3& up, Real trustPerp, Real trustPar, Real T) {
        const Real kp = std::exp(-(1.0 - trustPerp) * T / cfg_.decayTau);
        const Real kq = std::exp(-(1.0 - trustPar) * T / cfg_.decayTau);
        const Vec3 par = up * dot(w_, up);
        w_ = (w_ - par) * kp + par * kq;
    }
    Vec3 emit(Real T) {
        const Real speed = norm(w_) + actT_ + actY_;
        const Quat yPred = y_ * expMap(w_ * T);
        const Vec3 e = logMap(conj(yPred) * q_);
        const Real mu = 1.0 - std::exp(-(cfg_.outBwRest + cfg_.outBwPerSpeed * speed) * T);
        Vec3 out = w_ + e * (mu / T);
        if (cfg_.restDeadband > 0) out = out * smoothstep(cfg_.restDeadband, 2 * cfg_.restDeadband, norm(out));
        y_ = normalized(y_ * expMap(out * T));
        return out_ = out;
    }
};

}  // namespace vg

// =====================================================================================================
// SYNTHETIC 0-DELAY PACKET GENERATOR & HARDWARE ACCEL CLOCK HIJACKING
// =====================================================================================================

static constexpr float HARD_IRON_X = 93.76f;
static constexpr float HARD_IRON_Y = -29.09f;
static constexpr float HARD_IRON_Z = 967.01f;

static vg::VirtualGyro gyro_engine;
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

static int64_t g_last_gyro_ts = 0;
static uint64_t g_last_emitted_seq = 0;
static bool g_pending_synth = false;

static inline void feed_accel(const ASensorEvent& e) {
    gyro_engine.onAccel(e.timestamp, e.acceleration.x, e.acceleration.y, e.acceleration.z);
}
static inline void feed_mag_uncal(const ASensorEvent& e) {
    gyro_engine.onMag(e.timestamp,
                      e.uncalibrated_magnetic.x_uncalib - HARD_IRON_X,
                      e.uncalibrated_magnetic.y_uncalib - HARD_IRON_Y,
                      e.uncalibrated_magnetic.z_uncalib - HARD_IRON_Z);
}
static inline void feed_mag_cal(const ASensorEvent& e) {
    gyro_engine.onMag(e.timestamp, e.magnetic.x, e.magnetic.y, e.magnetic.z);
}

static inline void write_synthetic_gyro(ASensorEvent& dst, int32_t handle, int32_t type, float wx, float wy, float wz, int64_t base_ts) {
    memset(&dst, 0, sizeof(ASensorEvent));
    dst.version = sizeof(ASensorEvent);
    dst.sensor = handle;
    dst.type = type;
    dst.gyro.status = ASENSOR_STATUS_ACCURACY_HIGH;

    int64_t latest = gyro_engine.latest_ts.load(std::memory_order_relaxed);
    int64_t ts = (latest > base_ts) ? latest : base_ts;
    if (ts <= g_last_gyro_ts) ts = g_last_gyro_ts + 100000;
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
    g_hw_accel = acc;
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

    if (__builtin_expect(g_pending_synth && queue == g_gyro_queue.load(std::memory_order_relaxed) && count >= 1, 0)) {
        g_pending_synth = false;
        int32_t handle = g_gyro_handle.load(std::memory_order_relaxed);
        if (handle != -1) {
            const float wx = gyro_engine.out_x.load(std::memory_order_relaxed);
            const float wy = gyro_engine.out_y.load(std::memory_order_relaxed);
            const float wz = gyro_engine.out_z.load(std::memory_order_relaxed);
            write_synthetic_gyro(events[0], handle, g_gyro_type.load(std::memory_order_relaxed), wx, wy, wz, gyro_engine.latest_ts.load(std::memory_order_relaxed));
            return 1;
        }
    }

    ssize_t n = orig_getEvents(queue, events, count);
    if (n <= 0) return n;

    const int state = own_state.load(std::memory_order_acquire);
    if (__builtin_expect(state == 1, 1)) {
        const float wx = gyro_engine.out_x.load(std::memory_order_relaxed);
        const float wy = gyro_engine.out_y.load(std::memory_order_relaxed);
        const float wz = gyro_engine.out_z.load(std::memory_order_relaxed);
        const bool is_gyro_q = (queue == g_gyro_queue.load(std::memory_order_relaxed));
        const bool game_wants_acc = g_game_wants_accel_on_gyro_q.load(std::memory_order_relaxed);

        bool emitted_gyro_in_batch = false;
        int64_t batch_ts = 0;

        for (ssize_t i = 0; i < n; i++) {
            ASensorEvent& e = events[i];
            if (e.timestamp > batch_ts) batch_ts = e.timestamp;

            if (e.type == ASENSOR_TYPE_GYROSCOPE || e.type == 4 || e.type == ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED || e.type == 16) {
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
                g_last_emitted_seq = gyro_engine.seq.load(std::memory_order_relaxed);
            } else if (e.type == ASENSOR_TYPE_ACCELEROMETER && is_gyro_q && !game_wants_acc) {
                int32_t handle = g_gyro_handle.load(std::memory_order_relaxed);
                if (handle != -1) {
                    write_synthetic_gyro(e, handle, g_gyro_type.load(std::memory_order_relaxed), wx, wy, wz, e.timestamp);
                    emitted_gyro_in_batch = true;
                    g_last_emitted_seq = gyro_engine.seq.load(std::memory_order_relaxed);
                }
            }
        }

        uint64_t cur_seq = gyro_engine.seq.load(std::memory_order_acquire);
        if (is_gyro_q && !emitted_gyro_in_batch && cur_seq != g_last_emitted_seq) {
            int32_t handle = g_gyro_handle.load(std::memory_order_relaxed);
            if (handle != -1) {
                g_last_emitted_seq = cur_seq;
                if ((size_t)n < count) {
                    write_synthetic_gyro(events[n], handle, g_gyro_type.load(std::memory_order_relaxed), wx, wy, wz, batch_ts);
                    n++;
                } else {
                    g_pending_synth = true;
                }
            }
        }
        return n;
    }

    std::lock_guard<std::mutex> lock(vg_lock);
    for (ssize_t i = 0; i < n; i++) {
        ASensorEvent& e = events[i];
        if (e.type == ASENSOR_TYPE_ACCELEROMETER) {
            feed_accel(e);
        } else if (e.type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || e.type == 14) {
            feed_mag_uncal(e);
        } else if (e.type == ASENSOR_TYPE_GYROSCOPE || e.type == 4 || e.type == ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED || e.type == 16) {
            const float wx = gyro_engine.out_x.load(std::memory_order_relaxed);
            const float wy = gyro_engine.out_y.load(std::memory_order_relaxed);
            const float wz = gyro_engine.out_z.load(std::memory_order_relaxed);
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
