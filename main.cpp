#include <android/sensor.h>
#include <dlfcn.h>
#include <string.h>
#include <math.h> 
#include "dobby.h"
#include "zygisk.hpp"

// --- TUNABLE PARAMETERS ---
static const float SENSITIVITY = 1.0f; 
static const float NOISE_GATE = 0.003f;
static const float FILTER_COEFFICIENT = 0.95f; // Adjusts responsiveness vs stability[span_11](start_span)[span_11](end_span)

// Exact CPU X Factory Biases for MT6835
static const float HARD_IRON_X = 93.76f;
static const float HARD_IRON_Y = -29.09f;
static const float HARD_IRON_Z = 967.01f;

// --- STATE VARIABLES ---
static float accelReading[3] = {0.0f, 0.0f, 9.81f};
static float magReading[3] = {0.0f, 1.0f, 0.0f};

static float fusedRotationMatrix[9];
static bool isInitialized = false;

static float lastYaw = 0.0f;
static float lastPitch = 0.0f;
static int64_t last_ts = 0;
static float final_gyro[3] = {0.0f, 0.0f, 0.0f};

typedef ssize_t (*getEvents_t)(ASensorEventQueue*, ASensorEvent*, size_t);
static getEvents_t orig_getEvents = nullptr;
typedef int (*setEventRate_t)(ASensorEventQueue*, ASensor const*, int32_t);
static setEventRate_t orig_setEventRate = nullptr;

// Standard Helper method to filter out high-frequency noise/jitter[span_12](start_span)[span_12](end_span)[span_13](start_span)[span_13](end_span)
void lowPass(float input[3], float output[3]) {
    float alpha = 0.2f; 
    for (int i = 0; i < 3; i++) {
        output[i] = output[i] + alpha * (input[i] - output[i]);
    }
}

// Computes the raw rotation matrix directly from gravity and magnetic vectors[span_14](start_span)[span_14](end_span)
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

    // Android standard matrix structure (Rows are East, North, Gravity)
    R[0] = Ex; R[1] = Ey; R[2] = Ez;
    R[3] = Nx; R[4] = Ny; R[5] = Nz;
    R[6] = Ax; R[7] = Ay; R[8] = Az;
    return true;
}

void compute_sensor_fusion(int64_t ts) {
    float currentRotationMatrix[9];
    
    // Compute the raw rotation matrix[span_15](start_span)[span_15](end_span)[span_16](start_span)[span_16](end_span)
    bool success = getRotationMatrix(currentRotationMatrix, accelReading, magReading);

    if (success) {
        if (!isInitialized) {
            for (int i = 0; i < 9; i++) fusedRotationMatrix[i] = currentRotationMatrix[i];
            isInitialized = true;
        } else {
            // Blend the previous stable matrix with the new matrix smoothly[span_17](start_span)[span_17](end_span)[span_18](start_span)[span_18](end_span)
            for (int i = 0; i < 9; i++) {
                fusedRotationMatrix[i] = (FILTER_COEFFICIENT * fusedRotationMatrix[i]) + 
                                        ((1.0f - FILTER_COEFFICIENT) * currentRotationMatrix[i]);
            }
        }

        if (last_ts == 0) {
            last_ts = ts;
            // Initialize last angles to prevent start-up snapping
            float screenX = fusedRotationMatrix[2];
            float screenY = fusedRotationMatrix[5];
            float screenZ = fusedRotationMatrix[8];
            lastYaw = atan2(screenX, screenY);
            lastPitch = asin(fmax(fmin(-screenZ, 1.0f), -1.0f));
            return;
        }

        float dt = (ts - last_ts) / 1000000000.0f;
        if (dt > 0.001f && dt < 0.1f) {
            
            // DIRECT MATRIX PROJECTION
            // Extract the vector pointing straight out of the Screen of the phone[span_19](start_span)[span_19](end_span)
            // This is the third column of our 3x3 fused rotation matrix (M2, M5, M8)[span_20](start_span)[span_20](end_span)
            float screenX = fusedRotationMatrix[2];
            float screenY = fusedRotationMatrix[5];
            float screenZ = fusedRotationMatrix[8];

            // Project this vector onto the Earth's flat horizontal plane (X-Y plane)[span_21](start_span)[span_21](end_span)[span_22](start_span)[span_22](end_span)
            // Using atan2 completely eliminates the 90-degree divide-by-zero singularity[span_23](start_span)[span_23](end_span)[span_24](start_span)[span_24](end_span)!
            float cleanYaw = atan2(screenX, screenY); 

            // Extract pitch securely without hitting a hard wall[span_25](start_span)[span_25](end_span)
            // screenZ is how much the screen is pointing up to the sky or down to the dirt
            float cleanPitch = asin(fmax(fmin(-screenZ, 1.0f), -1.0f)); 

            // Calculate exact angular delta and wrap boundaries
            float dyaw = cleanYaw - lastYaw;
            while (dyaw > M_PI) dyaw -= 2.0f * M_PI;
            while (dyaw < -M_PI) dyaw += 2.0f * M_PI;
            
            float dpitch = cleanPitch - lastPitch;

            // Differentiate to angular velocity (rad/s) to simulate raw gyroscope[span_26](start_span)[span_26](end_span)
            float speed_yaw = dyaw / dt;
            float speed_pitch = dpitch / dt;

            // Map purely to CODM's isolated aiming axes
            final_gyro[0] = (fabs(speed_pitch) > NOISE_GATE) ? speed_pitch * SENSITIVITY : 0.0f;
            final_gyro[1] = (fabs(speed_yaw) > NOISE_GATE) ? speed_yaw * SENSITIVITY : 0.0f;
            final_gyro[2] = 0.0f; // Lock roll to kill racing-game tilt

            lastYaw = cleanYaw;
            lastPitch = cleanPitch;
        }
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
                // Apply low-pass filter to smooth out sudden bumps[span_27](start_span)[span_27](end_span)[span_28](start_span)[span_28](end_span)
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
