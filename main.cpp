#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
static const float SENSITIVITY = 1.0f; 
static const float NOISE_GATE = 0.003f;
static const float FILTER_COEFFICIENT = 0.85f; 

// Exact CPU X Factory Biases for MT6835
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

// --- STATE VARIABLES ---
static float accelReading[3] = {0.0f, 0.0f, 9.81f};
static float magReading[3] = {0.0f, 1.0f, 0.0f};

static float lastRotationMatrix[9] = {1,0,0, 0,1,0, 0,0,1};
static bool isInitialized = false;
static int64_t last_ts = 0;

static float smooth_gyro[3] = {0.0f, 0.0f, 0.0f};
static float final_gyro[3] = {0.0f, 0.0f, 0.0f};

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

// Hardware Noise Filter (Reduced filtering for faster response)
void lowPass(float input[3], float output[3]) {
    float alpha = 0.6f; // Changed from 0.2f to 0.6f
    for (int i = 0; i < 3; i++) {
        output[i] = output[i] + alpha * (input[i] - output[i]);
    }
}

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

void compute_sensor_fusion(int64_t ts) {
    float C[9];
    bool success = getRotationMatrix(C, accelReading, magReading);

    if (success) {
        if (!isInitialized) {
            memcpy(lastRotationMatrix, C, sizeof(float) * 9);
            isInitialized = true;
            last_ts = ts;
            return;
        }

        float dt = (ts - last_ts) / 1000000000.0f;
        if (dt > 0.001f && dt < 0.1f) {
            
            float L[9];
            memcpy(L, lastRotationMatrix, sizeof(float) * 9);
            float dR[9];

            dR[0] = L[0]*C[0] + L[3]*C[3] + L[6]*C[6];
            dR[1] = L[0]*C[1] + L[3]*C[4] + L[6]*C[7];
            dR[2] = L[0]*C[2] + L[3]*C[5] + L[6]*C[8];
            
            dR[3] = L[1]*C[0] + L[4]*C[3] + L[7]*C[6];
            dR[4] = L[1]*C[1] + L[4]*C[4] + L[7]*C[7];
            dR[5] = L[1]*C[2] + L[4]*C[5] + L[7]*C[8];
            
            dR[6] = L[2]*C[0] + L[5]*C[3] + L[8]*C[6];
            dR[7] = L[2]*C[1] + L[5]*C[4] + L[8]*C[7];
            dR[8] = L[2]*C[2] + L[5]*C[5] + L[8]*C[8];

            float raw_gyro_x = (dR[7] - dR[5]) / (2.0f * dt); 
            float raw_gyro_y = (dR[2] - dR[6]) / (2.0f * dt); 
            float raw_gyro_z = (dR[3] - dR[1]) / (2.0f * dt); 

            smooth_gyro[0] = (FILTER_COEFFICIENT * smooth_gyro[0]) + ((1.0f - FILTER_COEFFICIENT) * raw_gyro_x);
            smooth_gyro[1] = (FILTER_COEFFICIENT * smooth_gyro[1]) + ((1.0f - FILTER_COEFFICIENT) * raw_gyro_y);
            smooth_gyro[2] = (FILTER_COEFFICIENT * smooth_gyro[2]) + ((1.0f - FILTER_COEFFICIENT) * raw_gyro_z);

            final_gyro[0] = (fabs(smooth_gyro[0]) > NOISE_GATE) ? smooth_gyro[0] * SENSITIVITY : 0.0f;
            final_gyro[1] = (fabs(smooth_gyro[1]) > NOISE_GATE) ? smooth_gyro[1] * SENSITIVITY : 0.0f;
            final_gyro[2] = (fabs(smooth_gyro[2]) > NOISE_GATE) ? smooth_gyro[2] * SENSITIVITY : 0.0f;
        }
        
        memcpy(lastRotationMatrix, C, sizeof(float) * 9);
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
                lowPass(events[i].acceleration.v, accelReading);
                latest_ts = events[i].timestamp;
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED || events[i].type == 14) {
                float raw_mag[3] = {
                    events[i].uncalibrated_magnetic.x_uncalib - HARD_IRON_X,
                    events[i].uncalibrated_magnetic.y_uncalib - HARD_IRON_Y,
                    events[i].uncalibrated_magnetic.z_uncalib - HARD_IRON_Z
                };
                lowPass(raw_mag, magReading);
                if (!fusion_ready) latest_ts = events[i].timestamp; 
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_MAGNETIC_FIELD) {
                lowPass(events[i].magnetic.v, magReading);
                if (!fusion_ready) latest_ts = events[i].timestamp;
                fusion_ready = true;
            } 
            else if (events[i].type == ASENSOR_TYPE_GYROSCOPE) {
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
        if (target_get) {
            DobbyHook(target_get, (dobby_dummy_func_t)hook_ASensorEventQueue_getEvents, (dobby_dummy_func_t*)&orig_getEvents);
        }
        void* target_rate = dlsym(libandroid, "ASensorEventQueue_setEventRate");
        if (target_rate) {
            DobbyHook(target_rate, (dobby_dummy_func_t)hook_ASensorEventQueue_setEventRate, (dobby_dummy_func_t*)&orig_setEventRate);
        }
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
