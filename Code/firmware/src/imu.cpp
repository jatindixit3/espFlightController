#include "imu.h"
#include "config.h"
#include <Wire.h>
#include <math.h>
#include <string.h>

namespace {

// ---- MPU6050 registers ----
constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

// Full scale ranges chosen to avoid clipping during aggressive freestyle
// maneuvers (flips can exceed 250dps / several g, so we don't use the
// tightest/most-sensitive ranges even though they'd be less noisy).
constexpr float GYRO_SCALE_LSB_PER_DPS = 16.4f;  // FS_SEL=3 -> +/-2000 dps
constexpr float ACCEL_SCALE_LSB_PER_G = 4096.0f; // AFS_SEL=2 -> +/-8g

// Mahony filter gains.
constexpr float MAHONY_KP = 2.0f;
constexpr float MAHONY_KI = 0.005f;

bool g_calibrated = false;
float g_gyroBiasDegS[3] = {0, 0, 0};

float g_boardAlign[3][3] = {{1,0,0},{0,1,0},{0,0,1}}; // identity by default

float g_q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // w,x,y,z
float g_integralFB[3] = {0, 0, 0};

ImuSample g_sample;

bool writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool readBytes(uint8_t reg, uint8_t* buf, size_t len) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    size_t got = Wire.requestFrom((int)MPU_ADDR, (int)len, (int)true);
    if (got != len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

void applyAlignment(const float in[3], float out[3]) {
    for (int r = 0; r < 3; r++) {
        out[r] = g_boardAlign[r][0]*in[0] + g_boardAlign[r][1]*in[1] + g_boardAlign[r][2]*in[2];
    }
}

// Reads raw accel(g)/gyro(dps) into the provided arrays. Returns false on I2C fault.
bool readRaw(float accelG[3], float gyroDps[3]) {
    uint8_t buf[14];
    if (!readBytes(REG_ACCEL_XOUT_H, buf, 14)) return false;

    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
    // buf[6..7] = temperature, unused
    int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

    accelG[0] = ax / ACCEL_SCALE_LSB_PER_G;
    accelG[1] = ay / ACCEL_SCALE_LSB_PER_G;
    accelG[2] = az / ACCEL_SCALE_LSB_PER_G;

    gyroDps[0] = gx / GYRO_SCALE_LSB_PER_DPS;
    gyroDps[1] = gy / GYRO_SCALE_LSB_PER_DPS;
    gyroDps[2] = gz / GYRO_SCALE_LSB_PER_DPS;
    return true;
}

void mahonyUpdate(float gxRad, float gyRad, float gzRad,
                   float ax, float ay, float az, float dt) {
    // Skip the accel correction entirely if the accel reading isn't a sane
    // unit vector (e.g. during hard punches) - avoids injecting garbage
    // attitude correction under high linear acceleration.
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    bool haveValidAccel = norm > 0.5f && norm < 1.5f; // roughly 0.5g-1.5g

    float q0 = g_q[0], q1 = g_q[1], q2 = g_q[2], q3 = g_q[3];

    if (haveValidAccel) {
        ax /= norm; ay /= norm; az /= norm;

        float vx = 2.0f*(q1*q3 - q0*q2);
        float vy = 2.0f*(q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        float ex = (ay*vz - az*vy);
        float ey = (az*vx - ax*vz);
        float ez = (ax*vy - ay*vx);

        if (MAHONY_KI > 0.0f) {
            g_integralFB[0] += MAHONY_KI * ex * dt;
            g_integralFB[1] += MAHONY_KI * ey * dt;
            g_integralFB[2] += MAHONY_KI * ez * dt;
            gxRad += g_integralFB[0];
            gyRad += g_integralFB[1];
            gzRad += g_integralFB[2];
        }

        gxRad += MAHONY_KP * ex;
        gyRad += MAHONY_KP * ey;
        gzRad += MAHONY_KP * ez;
    }

    float qDot0 = 0.5f * (-q1*gxRad - q2*gyRad - q3*gzRad);
    float qDot1 = 0.5f * ( q0*gxRad + q2*gzRad - q3*gyRad);
    float qDot2 = 0.5f * ( q0*gyRad - q1*gzRad + q3*gxRad);
    float qDot3 = 0.5f * ( q0*gzRad + q1*gyRad - q2*gxRad);

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    float qNorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (qNorm < 1e-6f) qNorm = 1e-6f;
    g_q[0] = q0 / qNorm;
    g_q[1] = q1 / qNorm;
    g_q[2] = q2 / qNorm;
    g_q[3] = q3 / qNorm;
}

void quaternionToEulerDeg(float outDeg[3]) {
    float q0 = g_q[0], q1 = g_q[1], q2 = g_q[2], q3 = g_q[3];
    float roll = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    float sinp = 2.0f*(q0*q2 - q3*q1);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    float pitch = asinf(sinp);
    float yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
    outDeg[0] = roll * (180.0f / (float)M_PI);
    outDeg[1] = pitch * (180.0f / (float)M_PI);
    outDeg[2] = yaw * (180.0f / (float)M_PI);
}

} // namespace

bool imuInit() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);

    uint8_t whoAmI = 0;
    if (!readBytes(REG_WHO_AM_I, &whoAmI, 1)) return false;
    if (whoAmI != 0x68 && whoAmI != 0x70 && whoAmI != 0x72 && whoAmI != 0x98) {
        // Some MPU6050 clone silicon reports slightly different WHOAMI values;
        // if you get a wiring-fault failure here with a genuine MPU6050, check
        // your specific chip revision's datasheet before assuming it's dead.
        return false;
    }

    if (!writeReg(REG_PWR_MGMT_1, 0x01)) return false; // wake, PLL w/ X gyro ref
    delay(10);
    if (!writeReg(REG_CONFIG, 0x01)) return false;      // DLPF ~184Hz, 1kHz internal sample
    if (!writeReg(REG_SMPLRT_DIV, 0x00)) return false;  // sample rate = 1kHz / (1+0)
    if (!writeReg(REG_GYRO_CONFIG, 0x18)) return false; // FS_SEL=3 -> +/-2000dps
    if (!writeReg(REG_ACCEL_CONFIG, 0x10)) return false; // AFS_SEL=2 -> +/-8g

    memset(&g_sample, 0, sizeof(g_sample));
    g_q[0] = 1.0f; g_q[1] = g_q[2] = g_q[3] = 0.0f;
    return true;
}

void imuSetBoardAlignment(float rollDeg, float pitchDeg, float yawDeg) {
    float r = rollDeg * (float)M_PI / 180.0f;
    float p = pitchDeg * (float)M_PI / 180.0f;
    float y = yawDeg * (float)M_PI / 180.0f;

    float cr = cosf(r), sr = sinf(r);
    float cp = cosf(p), sp = sinf(p);
    float cy = cosf(y), sy = sinf(y);

    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    g_boardAlign[0][0] = cy*cp;
    g_boardAlign[0][1] = cy*sp*sr - sy*cr;
    g_boardAlign[0][2] = cy*sp*cr + sy*sr;
    g_boardAlign[1][0] = sy*cp;
    g_boardAlign[1][1] = sy*sp*sr + cy*cr;
    g_boardAlign[1][2] = sy*sp*cr - cy*sr;
    g_boardAlign[2][0] = -sp;
    g_boardAlign[2][1] = cp*sr;
    g_boardAlign[2][2] = cp*cr;
}

bool imuCalibrateGyro() {
    constexpr uint32_t MAX_TOTAL_MS = 12000; // overall giveup timeout
    constexpr uint32_t SAMPLE_PERIOD_MS = 5;

    uint32_t windowStart = millis();
    uint32_t totalStart = windowStart;
    double sum[3] = {0, 0, 0};
    uint32_t sampleCount = 0;

    while (true) {
        if (millis() - totalStart > MAX_TOTAL_MS) {
            g_calibrated = false;
            return false;
        }

        float accelG[3], gyroDps[3];
        if (!readRaw(accelG, gyroDps)) {
            g_calibrated = false;
            return false; // I2C fault - do not pretend calibration succeeded
        }

        float accelMag = sqrtf(accelG[0]*accelG[0] + accelG[1]*accelG[1] + accelG[2]*accelG[2]);
        float gyroMag = sqrtf(gyroDps[0]*gyroDps[0] + gyroDps[1]*gyroDps[1] + gyroDps[2]*gyroDps[2]);

        bool accelStill = fabsf(accelMag - 1.0f) < GYRO_CALIBRATION_STILLNESS_ACCEL_TOLERANCE_G;
        bool gyroStill = gyroMag < GYRO_CALIBRATION_STILLNESS_GYRO_TOLERANCE_DPS;

        if (!accelStill || !gyroStill) {
            // Board moved (or was never still) - discard everything accumulated
            // so far and restart the window. This is the accelerometer gate:
            // we refuse to accept a gyro bias unless the accelerometer agrees
            // the board was genuinely motionless the whole time.
            windowStart = millis();
            sum[0] = sum[1] = sum[2] = 0;
            sampleCount = 0;
            delay(SAMPLE_PERIOD_MS);
            continue;
        }

        sum[0] += gyroDps[0];
        sum[1] += gyroDps[1];
        sum[2] += gyroDps[2];
        sampleCount++;

        if (millis() - windowStart >= GYRO_CALIBRATION_DURATION_MS) {
            g_gyroBiasDegS[0] = (float)(sum[0] / sampleCount);
            g_gyroBiasDegS[1] = (float)(sum[1] / sampleCount);
            g_gyroBiasDegS[2] = (float)(sum[2] / sampleCount);
            g_calibrated = true;
            return true;
        }

        delay(SAMPLE_PERIOD_MS);
    }
}

bool imuIsCalibrated() {
    return g_calibrated;
}

void imuUpdate(float dtSeconds) {
    float accelG[3], gyroDps[3];
    if (!readRaw(accelG, gyroDps)) {
        // Sensor read failed this cycle - hold last attitude/rate rather than
        // injecting a zeroed reading, and let the caller's health check
        // (based on repeated failures) decide whether to trigger failsafe.
        return;
    }

    float gyroBiasCorrected[3] = {
        gyroDps[0] - g_gyroBiasDegS[0],
        gyroDps[1] - g_gyroBiasDegS[1],
        gyroDps[2] - g_gyroBiasDegS[2]
    };

    float alignedGyro[3], alignedAccel[3];
    applyAlignment(gyroBiasCorrected, alignedGyro);
    applyAlignment(accelG, alignedAccel);

    memcpy(g_sample.gyroDegS, alignedGyro, sizeof(alignedGyro));
    memcpy(g_sample.accelG, alignedAccel, sizeof(alignedAccel));

    float gxRad = alignedGyro[0] * (float)M_PI / 180.0f;
    float gyRad = alignedGyro[1] * (float)M_PI / 180.0f;
    float gzRad = alignedGyro[2] * (float)M_PI / 180.0f;

    mahonyUpdate(gxRad, gyRad, gzRad, alignedAccel[0], alignedAccel[1], alignedAccel[2], dtSeconds);
    quaternionToEulerDeg(g_sample.attitudeDeg);
    memcpy(g_sample.quaternion, g_q, sizeof(g_q));
}

const ImuSample& imuGetSample() {
    return g_sample;
}
