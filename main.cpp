#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
static const float SENSITIVITY = 1.0f; 
static const float NOISE_GATE = 0.005f;

// Low-pass filter coefficient for final angular rates[span_13](start_span)[span_13](end_span)
static const float ALPHA_GYRO = 0.2f; 
// Low-pass for incoming raw sensor noise to mitigate linear acceleration artifacts[span_14](start_span)[span_14](end_span)[span_15](start_span)[span_15](end_span)[span_16](start_span)[span_16](end_span)
static const float ALPHA_SENSOR = 0.2f; 

// Mandatory hard-iron calibration biases for MT6835[span_17](start_span)[span_17](end_span)
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

struct Quat { float x, y, z, w; };

static float accelReading[3] = {0.0f, 0.0f, 9.81f};
static float magReading[3] = {0.0f, 1.0f, 0.0f};

// State trackers for the differentiation step[span_18](start_span)[span_18](end_span)
static Quat prev_q = {0.0f, 0.0f, 0.0f, 0.0f};
static bool isInitialized = false;
static int64_t last_ts = 0;

static float prev_gyro[3] = {0.0f, 0.0f, 0.0f};
static float final_gyro[3] = {0.0f, 0.0f, 0.0f};
static float final_quat[4] = {0.0f, 0.0f, 0.0f, 1.0f};

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

void lowPass(float input[3], float output[3], float alpha) {
    for (int i = 0; i < 3; i++) {
        output[i] = output[i] + alpha * (input[i] - output[i]);
    }
}

// 1. Compute Orientation Matrix (TRIAD algorithm)[span_19](start_span)[span_19](end_span)[span_20](start_span)[span_20](end_span)
bool getRotationMatrix(float R[9], float gravity[3], float geomagnetic[3]) {
    float Ax = gravity[0], Ay = gravity[1], Az = gravity[2];
    float normA = sqrt(Ax*Ax + Ay*Ay + Az*Az);
    if (normA < 0.1f) return false;
    Ax /= normA; Ay /= normA; Az /= normA;

    float Ex = geomagnetic[1] * Az - geomagnetic[2] * Ay;
    float Ey = geomagnetic[2] * Ax - geomagnetic[0] * Az;
    float Ez = geomagnetic[0] * Ay - geomagnetic[1] * Ax;
    float normE = sqrt(Ex*Ex + Ey*Ey + Ez*Ez);
    if (normE < 0.1f) return false;
    Ex /= normE; Ey /= normE; Ez /= normE;

    float Nx = Ay * Ez - Az * Ey;
    float Ny = Az * Ex - Ax * Ez;
    float Nz = Ax * Ey - Ay * Ex;

    R[0] = Ex; R[1] = Ey; R[2] = Ez;
    R[3] = Nx; R[4] = Ny; R[5] = Nz;
    R[6] = Ax; R[7] = Ay; R[8] = Az;
    return true;
}

// 2. Convert to Quaternion to avoid gimbal lock[span_21](start_span)[span_21](end_span)
Quat matrixToQuaternion(float R[9]) {
    Quat q;
    float trace = R[0] + R[4] + R[8];
    if (trace > 0.0f) {
        float s = sqrt(trace + 1.0f) * 2.0f;
        q.w = 0.25f * s;         
        q.x = (R[7] - R[5]) / s; 
        q.y = (R[2] - R[6]) / s; 
        q.z = (R[3] - R[1]) / s; 
    } else if ((R[0] > R[4]) && (R[0] > R[8])) {
        float s = sqrt(1.0f + R[0] - R[4] - R[8]) * 2.0f;
        q.w = (R[7] - R[5]) / s;
        q.x = 0.25f * s;
        q.y = (R[3] + R[1]) / s;
        q.z = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
        float s = sqrt(1.0f + R[4] - R[0] - R[8]) * 2.0f;
        q.w = (R[2] - R[6]) / s;
        q.x = (R[3] + R[1]) / s;
        q.y = 0.25f * s;
        q.z = (R[7] + R[5]) / s;
    } else {
        float s = sqrt(1.0f + R[8] - R[0] - R[4]) * 2.0f;
        q.w = (R[3] - R[1]) / s;
        q.x = (R[2] + R[6]) / s;
        q.y = (R[7] + R[5]) / s;
        q.z = 0.25f * s;
    }
    
    // Ensure unit quaternion length
    float norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    q.x /= norm; q.y /= norm; q.z /= norm; q.w /= norm;
    return q;
}

// Quaternion multiplication helper[span_22](start_span)[span_22](end_span)
Quat q_mult(Quat q1, Quat q2) {
    Quat result;
    result.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    result.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    result.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    result.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return result;
}

void compute_sensor_fusion(int64_t ts) {
    float R[9];
    bool success = getRotationMatrix(R, accelReading, magReading);

    if (success) {
        Quat current_q = matrixToQuaternion(R);
        
        // Export to Android Rotation Vector formats
        final_quat[0] = current_q.x;
        final_quat[1] = current_q.y;
        final_quat[2] = current_q.z;
        final_quat[3] = current_q.w;

        if (!isInitialized || last_ts == 0) {
            prev_q = current_q;
            isInitialized = true;
            last_ts = ts;
            return;
        }

        float dt = (ts - last_ts) / 1000000000.0f;
        if (dt > 0.001f && dt < 0.1f) {
            
            // Shortest path validation to prevent math flips
            float dot = current_q.x*prev_q.x + current_q.y*prev_q.y + current_q.z*prev_q.z + current_q.w*prev_q.w;
            if (dot < 0.0f) {
                current_q.x = -current_q.x; current_q.y = -current_q.y; 
                current_q.z = -current_q.z; current_q.w = -current_q.w;
            }

            // 3. Compute Quaternion derivative (dq = (current_q - prev_q) / dt)[span_23](start_span)[span_23](end_span)[span_24](start_span)[span_24](end_span)
            Quat dq;
            dq.x = (current_q.x - prev_q.x) / dt;
            dq.y = (current_q.y - prev_q.y) / dt;
            dq.z = (current_q.z - prev_q.z) / dt;
            dq.w = (current_q.w - prev_q.w) / dt;

            // Calculate inverse of previous quaternion[span_25](start_span)[span_25](end_span)
            Quat q_inv = {-prev_q.x, -prev_q.y, -prev_q.z, prev_q.w};

            // 4. Calculate raw angular velocity (w = 2 * q_inv * dq)[span_26](start_span)[span_26](end_span)[span_27](start_span)[span_27](end_span)
            Quat w_q = q_mult(q_inv, dq);
            
            float raw_gyro[3] = {
                2.0f * w_q.x,
                2.0f * w_q.y,
                2.0f * w_q.z
            };

            // 5. Apply low-pass filter to smooth out derivative noise[span_28](start_span)[span_28](end_span)[span_29](start_span)[span_29](end_span)
            prev_gyro[0] = (ALPHA_GYRO * raw_gyro[0]) + ((1.0f - ALPHA_GYRO) * prev_gyro[0]);
            prev_gyro[1] = (ALPHA_GYRO * raw_gyro[1]) + ((1.0f - ALPHA_GYRO) * prev_gyro[1]);
            prev_gyro[2] = (ALPHA_GYRO * raw_gyro[2]) + ((1.0f - ALPHA_GYRO) * prev_gyro[2]);

            // Final output mapping with noise gate
            final_gyro[0] = (fabs(prev_gyro[0]) > NOISE_GATE) ? prev_gyro[0] * SENSITIVITY : 0.0f;
            final_gyro[1] = (fabs(prev_gyro[1]) > NOISE_GATE) ? prev_gyro[1] * SENSITIVITY : 0.0f;
            final_gyro[2] = (fabs(prev_gyro[2]) > NOISE_GATE) ? prev_gyro[2] * SENSITIVITY : 0.0f;
        }
        
        // Save states for next step[span_30](start_span)[span_30](end_span)
        prev_q = current_q;
        last_ts = ts;
    }
}

ssize_t hook_ASensorEventQueue_getEvents(ASensorEventQueue* queue, ASensorEvent* events, size_t count) {
    ssize_t actual_events = orig_getEvents(queue, events, count);
    if (actual_events > 0) {
        bool fusion_ready = false;
        int64_t latest_ts = 0;

        for (ssize_t i = 0; i < actual_events; i++) {
            if (events[i].type == ASENSOR_TYPE_ACCELEROMETER) {
                lowPass(events[i].acceleration.v, accelReading, ALPHA_SENSOR);
                latest_ts = events[i].timestamp;
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                float raw_mag[3] = {
                    events[i].uncalibrated_magnetic.x_uncalib - HARD_IRON_X,
                    events[i].uncalibrated_magnetic.y_uncalib - HARD_IRON_Y,
                    events[i].uncalibrated_magnetic.z_uncalib - HARD_IRON_Z
                };
                lowPass(raw_mag, magReading, ALPHA_SENSOR);
                if (!fusion_ready) latest_ts = events[i].timestamp; 
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) {
                lowPass(events[i].magnetic.v, magReading, ALPHA_SENSOR);
                if (!fusion_ready) latest_ts = events[i].timestamp;
                fusion_ready = true;
            } 
            
            // Overwrite ALL Gyro and Rotation Vector Events to ensure CoD Mobile uses our clean math
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE || events[i].type == 4) {
                events[i].vector.x = final_gyro[0]; 
                events[i].vector.y = final_gyro[1]; 
                events[i].vector.z = final_gyro[2]; 
            }
            else if (events[i].type == ASENSOR_TYPE_ROTATION_VECTOR || 
                     events[i].type == ASENSOR_TYPE_GAME_ROTATION_VECTOR || 
                     events[i].type == 11 || events[i].type == 15) {
                events[i].data[0] = final_quat[0]; // x
                events[i].data[1] = final_quat[1]; // y
                events[i].data[2] = final_quat[2]; // z
                events[i].data[3] = final_quat[3]; // w
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
