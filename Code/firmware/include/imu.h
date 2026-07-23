#pragma once
#include <stdint.h>

// ============================================================================
// MPU6050 driver + Mahony gyro/accel sensor fusion.
//
// Calibration is gated on the accelerometer confirming the board is actually
// still (see imuCalibrateGyro): gyro bias is only accepted if, throughout the
// sampling window, the accel vector reads ~1g with low variance. If the board
// is moved during calibration, it is rejected and retried rather than baking
// in a bad bias - this is what prevents long-term drift from ruining hover.
// ============================================================================

struct ImuSample {
    float gyroDegS[3];   // roll, pitch, yaw rate, deg/s, bias-corrected
    float accelG[3];     // x, y, z, g
    float attitudeDeg[3]; // roll, pitch, yaw, degrees (fused estimate)
    float quaternion[4]; // w, x, y, z
};

// Brings up I2C and the MPU6050, configures full-scale ranges and DLPF.
// Returns false if the sensor does not ACK / WHO_AM_I mismatches (wiring fault).
bool imuInit();

// Blocking gyro bias calibration, gated on accelerometer-confirmed stillness.
// Safe to call repeatedly (e.g. re-triggered from the configurator). Returns
// false if stillness could not be confirmed within a generous timeout - the
// caller must not allow arming in that case.
bool imuCalibrateGyro();

bool imuIsCalibrated();

// Reads the sensor and advances the fusion filter by dt seconds. Call once per
// flight loop iteration.
void imuUpdate(float dtSeconds);

const ImuSample& imuGetSample();

// Applies the configured board-mounting alignment (settings.boardAlign*Deg) to
// raw sensor axes to obtain the "as if perfectly mounted" quad axes.
void imuSetBoardAlignment(float rollDeg, float pitchDeg, float yawDeg);
