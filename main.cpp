#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <cmath>
#include "dobby.h"
#include "zygisk.hpp"

// VirtualGyro - Ported to C++ for Zygisk
// Pure Kalman-style tracking filter. Zero Euler angles. Zero Gimbal Lock.
// Natively outputs true 3D hardware velocity; the game handles display rotation remap natively.

class VirtualGyro {
public:
    static constexpr double G = 9.80665;

    // ---- TUNING (Direct from Java source) ----
    double sigmaTilt = 0.03;                    
    double sigmaHead = 0.03;                    
    double handAcc = 12.0;                      
    double gTol = 2.0;                          
    double magTol = 0.15;                       
    double dipTol = 8.0 * M_PI / 180.0;         
    double jumpReject = 30.0 * M_PI / 180.0;    
    double outageTau = 0.15;                    
    double deadzone = 1.5 * M_PI / 180.0;       

    // ---- STATE ----
    bool is_init = false;
    double q[4] = {1.0, 0.0, 0.0, 0.0};         
    double w[3] = {0.0, 0.0, 0.0};              
    double t = NAN;
    bool haveA = false, haveM = false;
    double ta0 = NAN, tm0 = NAN;
    double a0[3] = {0}, m0[3] = {0};
    double tAcc = NAN, tMag = NAN;
    double mRef = 0, dipRef = 0;
    double hRefX = 0.0, hRefY = 1.0;
    double rejectT = 0.0;
    double R[9] = {0};                          

    // ------------------------------------------------------------------ helpers
    static double interval(double ts, double prev) {
        return std::isnan(prev) ? 0.01 : std::fmin(0.1, std::fmax(1e-3, ts - prev));
    }

    static double softGate(double dev, double tol) {
        double x = dev / tol;
        return x >= 1.0 ? 0.0 : 1.0 - x * x;
    }

    static void trackingGains(double T, double sigmaMeas, double sigmaAcc, double out[2]) {
        double lam = sigmaAcc * T * T / sigmaMeas;
        double r = (4.0 + lam - std::sqrt(8.0 * lam + lam * lam)) / 4.0;
        double alpha = 1.0 - r * r;
        double beta = 2.0 * (2.0 - alpha) - 4.0 * r;
        out[0] = std::fmin(alpha, 0.9);
        out[1] = std::fmin(std::fmax(beta, 0.0), 0.8);
    }

    static void rotvecBetween(double ux, double uy, double uz, double vx, double vy, double vz, double out[3]) {
        double cx = uy * vz - uz * vy, cy = uz * vx - ux * vz, cz = ux * vy - uy * vx;
        double s = std::sqrt(cx * cx + cy * cy + cz * cz);
        if (s < 1e-12) { out[0] = out[1] = out[2] = 0.0; return; }
        double k = std::atan2(s, ux * vx + uy * vy + uz * vz) / s;
        out[0] = cx * k; out[1] = cy * k; out[2] = cz * k;
    }

    static void qMul(const double a[4], const double b[4], double out[4]) {
        out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
        out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
        out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
        out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    }

    static void qNormalize(double q[4]) {
        double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        if(n > 1e-9) { q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n; }
    }

    static void qExp(double x, double y, double z, double out[4]) {
        double ang = std::sqrt(x * x + y * y + z * z);
        if (ang < 1e-9) {
            out[0] = 1.0; out[1] = 0.5 * x; out[2] = 0.5 * y; out[3] = 0.5 * z;
            qNormalize(out);
            return;
        }
        double s = std::sin(0.5 * ang) / ang;
        out[0] = std::cos(0.5 * ang);
        out[1] = s * x; out[2] = s * y; out[3] = s * z;
    }

    static void toMatrix(const double q[4], double m[9]) {
        double w_ = q[0], x = q[1], y = q[2], z = q[3];
        m[0] = 1 - 2 * (y * y + z * z); m[1] = 2 * (x * y - z * w_);     m[2] = 2 * (x * z + y * w_);
        m[3] = 2 * (x * y + z * w_);     m[4] = 1 - 2 * (x * x + z * z); m[5] = 2 * (y * z - x * w_);
        m[6] = 2 * (x * z - y * w_);     m[7] = 2 * (y * z + x * w_);     m[8] = 1 - 2 * (x * x + y * y);
    }

    static void matrixToQ(const double m[9], double out[4]) {
        double tr = m[0] + m[4] + m[8];
        if (tr > 0) {
            double s = std::sqrt(tr + 1.0) * 2;
            out[0] = 0.25 * s; out[1] = (m[7] - m[5]) / s; out[2] = (m[2] - m[6]) / s; out[3] = (m[3] - m[1]) / s;
        } else if (m[0] > m[4] && m[0] > m[8]) {
            double s = std::sqrt(1.0 + m[0] - m[4] - m[8]) * 2;
            out[0] = (m[7] - m[5]) / s; out[1] = 0.25 * s; out[2] = (m[1] + m[3]) / s; out[3] = (m[2] + m[6]) / s;
        } else if (m[4] > m[8]) {
            double s = std::sqrt(1.0 + m[4] - m[0] - m[8]) * 2;
            out[0] = (m[2] - m[6]) / s; out[1] = (m[1] + m[3]) / s; out[2] = 0.25 * s; out[3] = (m[5] + m[7]) / s;
        } else {
            double s = std::sqrt(1.0 + m[8] - m[0] - m[4]) * 2;
            out[0] = (m[3] - m[1]) / s; out[1] = (m[2] + m[6]) / s; out[2] = (m[5] + m[7]) / s; out[3] = 0.25 * s;
        }
    }

    // ------------------------------------------------------------------ internals
    void correct(double ex, double ey, double ez, double alpha, double beta, double T) {
        double exp_q[4], tmp_q[4];
        qExp(alpha * ex, alpha * ey, alpha * ez, exp_q);
        qMul(q, exp_q, tmp_q);
        q[0] = tmp_q[0]; q[1] = tmp_q[1]; q[2] = tmp_q[2]; q[3] = tmp_q[3];
        qNormalize(q);
        double k = beta / T;
        w[0] += k * ex; w[1] += k * ey; w[2] += k * ez;
    }

    void decay(double gx, double gy, double gz, double T, bool along) {
        double k = std::exp(-T / outageTau);
        double d = w[0] * gx + w[1] * gy + w[2] * gz;
        double px = d * gx, py = d * gy, pz = d * gz;               
        if (along) {
            w[0] = px * k + (w[0] - px); w[1] = py * k + (w[1] - py); w[2] = pz * k + (w[2] - pz);
        } else {
            w[0] = px + k * (w[0] - px); w[1] = py + k * (w[1] - py); w[2] = pz + k * (w[2] - pz);
        }
    }

    void advance(double ts) {
        double dt = ts - t;
        if (!(dt > 0)) return;
        if (dt > 0.25) {                                            
            w[0] = w[1] = w[2] = 0.0;
        } else {
            double exp_q[4], tmp_q[4];
            qExp(w[0] * dt, w[1] * dt, w[2] * dt, exp_q);
            qMul(q, exp_q, tmp_q);
            q[0] = tmp_q[0]; q[1] = tmp_q[1]; q[2] = tmp_q[2]; q[3] = tmp_q[3];
            qNormalize(q);
        }
        t = ts;
    }

    void tryInit() {
        if (!haveA || !haveM) return;
        double an = std::sqrt(a0[0]*a0[0] + a0[1]*a0[1] + a0[2]*a0[2]);
        double ux = a0[0]/an, uy = a0[1]/an, uz = a0[2]/an;
        double ex = m0[1]*a0[2] - m0[2]*a0[1];
        double ey = m0[2]*a0[0] - m0[0]*a0[2];
        double ez = m0[0]*a0[1] - m0[1]*a0[0];
        double en = std::sqrt(ex*ex + ey*ey + ez*ez);
        if (en < 1e-9) return;                                      
        ex /= en; ey /= en; ez /= en;
        double nx = uy*ez - uz*ey, ny = uz*ex - ux*ez, nz = ux*ey - uy*ex;
        double M[9] = {ex, ey, ez, nx, ny, nz, ux, uy, uz};
        matrixToQ(M, q);
        qNormalize(q);
        t = std::fmax(ta0, tm0);
        double mn = std::sqrt(m0[0]*m0[0] + m0[1]*m0[1] + m0[2]*m0[2]);
        mRef = mn;
        dipRef = std::acos((m0[0]*a0[0] + m0[1]*a0[1] + m0[2]*a0[2]) / (mn * an));
        hRefX = 0.0; hRefY = 1.0;                                   
        is_init = true;
    }

    // ------------------------------------------------------------------ inputs
    void onAccel(double ts, double ax, double ay, double az) {
        if (!is_init) {
            a0[0] = ax; a0[1] = ay; a0[2] = az; ta0 = ts; haveA = true;
            tryInit();
            return;
        }
        double T = interval(ts, tAcc);
        tAcc = ts;
        advance(ts);
        double n = std::sqrt(ax * ax + ay * ay + az * az);
        if (n < 1e-3) return;                                       
        toMatrix(q, R);
        double gx = R[6], gy = R[7], gz = R[8];                     
        double e[3];
        rotvecBetween(ax / n, ay / n, az / n, gx, gy, gz, e);
        double wgt = softGate(std::abs(n - G), gTol);
        if (wgt < 0.02) {                                           
            decay(gx, gy, gz, T, false);
            return;
        }
        double gains[2];
        trackingGains(T, sigmaTilt / std::sqrt(wgt), handAcc, gains);
        correct(e[0], e[1], e[2], gains[0], gains[1], T);
    }

    void onMag(double ts, double mx, double my, double mz) {
        if (!is_init) {
            m0[0] = mx; m0[1] = my; m0[2] = mz; tm0 = ts; haveM = true;
            tryInit();
            return;
        }
        double T = interval(ts, tMag);
        tMag = ts;
        advance(ts);
        double nm = std::sqrt(mx * mx + my * my + mz * mz);
        if (nm < 1e-3) return;
        toMatrix(q, R);
        double gx = R[6], gy = R[7], gz = R[8];
        double hx = R[0] * mx + R[1] * my + R[2] * mz;              
        double hy = R[3] * mx + R[4] * my + R[5] * mz;              
        double hn = std::sqrt(hx * hx + hy * hy);
        double c = (mx * gx + my * gy + mz * gz) / nm;
        double dip = std::acos(std::fmax(-1.0, std::fmin(1.0, c)));
        double wgt = softGate(std::abs(nm - mRef) / mRef, magTol) * softGate(std::abs(dip - dipRef), dipTol);
        if (hn < 0.2 * nm) wgt = 0.0;                               
        double psi = 0.0;
        if (wgt >= 0.02) {
            psi = std::atan2(hx * hRefY - hy * hRefX, hx * hRefX + hy * hRefY);
            if (std::abs(psi) > jumpReject) wgt = 0.0;              
        }
        if (wgt < 0.02) {
            rejectT += T;
            if (rejectT > 0.5 && hn >= 0.2 * nm) {                  
                hRefX = hx / hn; hRefY = hy / hn;
                mRef = nm; dipRef = dip; rejectT = 0.0;
            }
            decay(gx, gy, gz, T, true);                             
            return;
        }
        rejectT = 0.0;
        double gains[2];
        trackingGains(T, sigmaHead / std::sqrt(wgt), handAcc, gains);
        correct(psi * gx, psi * gy, psi * gz, gains[0], gains[1], T);   
        if (wgt > 0.9) {                                            
            double k = std::fmin(1.0, T / 20.0);
            mRef += k * (nm - mRef);
            dipRef += k * (dip - dipRef);
        }
    }

    // ------------------------------------------------------------------ outputs
    void getRate(float out[3]) {
        if (!is_init) { out[0] = out[1] = out[2] = 0.0f; return; }
        double n = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
        if (n <= deadzone) { out[0] = out[1] = out[2] = 0.0f; return; }
        double s = 1.0 - deadzone / n;
        out[0] = (float)(w[0] * s);
        out[1] = (float)(w[1] * s);
        out[2] = (float)(w[2] * s);
    }
};

// --- GLOBAL INSTANCE & CONFIG ---
static VirtualGyro vgyro;
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t n = orig_getEvents(queue, events, count);
    if (n <= 0) return n;

    // PASS 1: Feed the Kalman Engine as events arrive natively
    for (ssize_t i = 0; i < n; i++) {                          
        double ts_sec = events[i].timestamp / 1e9; // Convert nano to sec for engine
        
        if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
            vgyro.onAccel(ts_sec, events[i].acceleration.x, events[i].acceleration.y, events[i].acceleration.z);
        } 
        // Feed Uncalibrated Mag with your Custom Hard-Iron subtraction
        else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
            float mx = events[i].uncalibrated_magnetic.x_uncalib - HARD_IRON_X;
            float my = events[i].uncalibrated_magnetic.y_uncalib - HARD_IRON_Y;
            float mz = events[i].uncalibrated_magnetic.z_uncalib - HARD_IRON_Z;
            vgyro.onMag(ts_sec, mx, my, mz);
        }
    }

    // PASS 2: Output flawless true hardware velocity to the Gyro
    for (ssize_t i = 0; i < n; i++) {                          
        if (events[i].type == ASENSOR_TYPE_GYROSCOPE || events[i].type == 4 || events[i].type == 16) {
            float rate[3];
            vgyro.getRate(rate);
            
            // True 3D velocity mapping; Android Display framework rotates it automatically for CODM
            events[i].vector.x = rate[0];
            events[i].vector.y = rate[1];
            events[i].vector.z = rate[2]; 
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
