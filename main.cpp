#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- HARDWARE CALIBRATION ---
// Your exact MT6835 Hard-Iron Biases
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

// --- STATE TRACKERS ---
static float accel_data[3] = {0.0f, 0.0f, 9.81f};
static float mag_data[3] = {0.0f, 1.0f, 0.0f};

// Quaternion state: [W, X, Y, Z]
static float prev_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float final_gyro[3] = {0.0f, 0.0f, 0.0f};

static int64_t prev_ts = 0;
static bool is_initialized = false;

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

// AOSP Rotation Matrix Generation
bool calculateRotationMatrix(float R[9], float A[3], float E[3]) {
    float normA = sqrt(A[0]*A[0] + A[1]*A[1] + A[2]*A[2]);
    if (normA < 0.1f) return false;
    float ax = A[0]/normA, ay = A[1]/normA, az = A[2]/normA;

    // Cross Product: Mag x Accel = "East" vector
    float Hx = E[1]*az - E[2]*ay;
    float Hy = E[2]*ax - E[0]*az;
    float Hz = E[0]*ay - E[1]*ax;
    float normH = sqrt(Hx*Hx + Hy*Hy + Hz*Hz);
    if (normH < 0.1f) return false;
    Hx /= normH; Hy /= normH; Hz /= normH;

    // Cross Product: Accel x East = "North" vector
    float Mx = ay*Hz - az*Hy;
    float My = az*Hx - ax*Hz;
    float Mz = ax*Hy - ay*Hx;

    R[0] = Hx; R[1] = Hy; R[2] = Hz;
    R[3] = Mx; R[4] = My; R[5] = Mz;
    R[6] = ax; R[7] = ay; R[8] = az;
    return true;
}

// Convert Rotation Matrix strictly to Quaternion [W, X, Y, Z]
void matrixToQuaternion(float R[9], float q[4]) {
    float trace = R[0] + R[4] + R[8];
    if (trace > 0.0f) {
        float s = sqrt(trace + 1.0f) * 2.0f;
        q[0] = 0.25f * s; // W
        q[1] = (R[7] - R[5]) / s; // X
        q[2] = (R[2] - R[6]) / s; // Y
        q[3] = (R[3] - R[1]) / s; // Z
    } else if ((R[0] > R[4]) && (R[0] > R[8])) {
        float s = sqrt(1.0f + R[0] - R[4] - R[8]) * 2.0f;
        q[0] = (R[7] - R[5]) / s; // W
        q[1] = 0.25f * s; // X
        q[2] = (R[3] + R[1]) / s; // Y
        q[3] = (R[2] + R[6]) / s; // Z
    } else if (R[4] > R[8]) {
        float s = sqrt(1.0f + R[4] - R[0] - R[8]) * 2.0f;
        q[0] = (R[2] - R[6]) / s; // W
        q[1] = (R[3] + R[1]) / s; // X
        q[2] = 0.25f * s; // Y
        q[3] = (R[7] + R[5]) / s; // Z
    } else {
        float s = sqrt(1.0f + R[8] - R[0] - R[4]) * 2.0f;
        q[0] = (R[3] - R[1]) / s; // W
        q[1] = (R[2] + R[6]) / s; // X
        q[2] = (R[7] + R[5]) / s; // Y
        q[3] = 0.25f * s; // Z
    }
}

// The Core Kinematic Engine
void compute_pure_kinematics(int64_t timestamp) {
    float R[9];
    float curr_q[4];

    if (!calculateRotationMatrix(R, accel_data, mag_data)) return;
    matrixToQuaternion(R, curr_q);

    if (!is_initialized) {
        prev_q[0] = curr_q[0]; prev_q[1] = curr_q[1];
        prev_q[2] = curr_q[2]; prev_q[3] = curr_q[3];
        prev_ts = timestamp;
        is_initialized = true;
        return;
    }

    float dt = (timestamp - prev_ts) / 1000000000.0f;
    if (dt <= 0.0f || dt > 0.2f) dt = 0.016f; // Fallback to 60hz frame time

    // Quaternion Double Cover Check (Prevents random 180-degree snapping)
    float dot_product = (prev_q[0] * curr_q[0]) + (prev_q[1] * curr_q[1]) + 
                        (prev_q[2] * curr_q[2]) + (prev_q[3] * curr_q[3]);
    if (dot_product < 0.0f) {
        curr_q[0] = -curr_q[0]; curr_q[1] = -curr_q[1];
        curr_q[2] = -curr_q[2]; curr_q[3] = -curr_q[3];
    }

    // Calculate Delta Quaternion: q_delta = prev_q^-1 * curr_q
    // prev_q^-1 is simply [W, -X, -Y, -Z]
    float dw = prev_q[0]*curr_q[0] - (-prev_q[1])*curr_q[1] - (-prev_q[2])*curr_q[2] - (-prev_q[3])*curr_q[3];
    float dx = prev_q[0]*curr_q[1] + (-prev_q[1])*curr_q[0] + (-prev_q[2])*curr_q[3] - (-prev_q[3])*curr_q[2];
    float dy = prev_q[0]*curr_q[2] - (-prev_q[1])*curr_q[3] + (-prev_q[2])*curr_q[0] + (-prev_q[3])*curr_q[1];
    float dz = prev_q[0]*curr_q[3] + (-prev_q[1])*curr_q[2] - (-prev_q[2])*curr_q[1] + (-prev_q[3])*curr_q[0];

    // Extract Pure Angular Velocity (rad/s)
    // Formula: omega = 2 * (q_delta_vector) / dt
    final_gyro[0] = (2.0f * dx) / dt;
    final_gyro[1] = (2.0f * dy) / dt;
    final_gyro[2] = (2.0f * dz) / dt;

    // Save state for the next frame
    prev_q[0] = curr_q[0]; prev_q[1] = curr_q[1];
    prev_q[2] = curr_q[2]; prev_q[3] = curr_q[3];
    prev_ts = timestamp;
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        bool fusion_ready = false;
        int64_t latest_ts = 0;

        for (ssize_t i = 0; i < actual_events; i++) {
            
            // 1. Gather raw unadulterated hardware data
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                accel_data[0] = events[i].acceleration.x;
                accel_data[1] = events[i].acceleration.y;
                accel_data[2] = events[i].acceleration.z;
                latest_ts = events[i].timestamp;
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                mag_data[0] = events[i].uncalibrated_magnetic.x_uncalib - HARD_IRON_X;
                mag_data[1] = events[i].uncalibrated_magnetic.y_uncalib - HARD_IRON_Y;
                mag_data[2] = events[i].uncalibrated_magnetic.z_uncalib - HARD_IRON_Z;
                if (!fusion_ready) latest_ts = events[i].timestamp;
                fusion_ready = true;
            }
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) {
                mag_data[0] = events[i].magnetic.x;
                mag_data[1] = events[i].magnetic.y;
                mag_data[2] = events[i].magnetic.z;
                if (!fusion_ready) latest_ts = events[i].timestamp;
                fusion_ready = true;
            }
            
            // 2. Feed our Custom Delta-Quaternion Engine to the game
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE || events[i].type == 4 || events[i].type == 16) {
                events[i].vector.x = final_gyro[0]; 
                events[i].vector.y = final_gyro[1]; 
                events[i].vector.z = final_gyro[2]; 
            }
            
            // 3. Hijack Rotation Vector (Forces CODM to trust our Quaternion state, not MediaTek's)
            else if (events[i].type == ASENSOR_TYPE_ROTATION_VECTOR || events[i].type == 11 || events[i].type == 15) {
                // Android uses [X, Y, Z, W] format for sensor events
                events[i].data[0] = prev_q[1]; // X
                events[i].data[1] = prev_q[2]; // Y
                events[i].data[2] = prev_q[3]; // Z
                events[i].data[3] = prev_q[0]; // W
            }
        }

        // Fire the physics calculation once all sensor data in the current poll is collected
        if (fusion_ready && latest_ts > 0) {
            compute_pure_kinematics(latest_ts);
        }
    }
    return actual_events;
}

int hook_ASensorEventQueue_setEventRate(ASensorEventQueue* queue, ASensor const* sensor, int32_t usec) {
    // Override MediaTek polling limitations, request maximum hardware speed (0 microseconds delay)
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
        // Inject exclusively into Call of Duty Mobile
        if (process && strcmp(process, "com.activision.callofduty.shooter") == 0) { 
            enable_hack = true; 
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
