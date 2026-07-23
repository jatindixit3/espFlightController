#pragma once
#include <stdint.h>

// ============================================================================
// Rate-mode PID with feedforward, D-term/gyro lowpass filtering, and an
// RPM-tracking notch filter sourced from ESC telemetry (see dshot.h for why
// RPM comes from the telemetry wire rather than bidirectional DShot decode).
// Angle/horizon leveling uses the Mahony-fused attitude estimate from imu.h.
// ============================================================================

enum FlightMode : uint8_t {
    FLIGHT_MODE_ACRO = 0,
    FLIGHT_MODE_ANGLE,
    FLIGHT_MODE_HORIZON
};

struct PidOutput {
    float roll;
    float pitch;
    float yaw;
};

void pidInit();

// rcCommand: roll/pitch/yaw stick deflection, -1..1 (deadband already applied
// by the caller). Reads gyro/attitude from imu.h and RPM from esc_telemetry.h
// internally.
PidOutput pidUpdate(const float rcCommand[3], FlightMode mode, float dtSeconds);

// Call when disarming, and once right after arming, to prevent integral
// windup/D-term spikes carrying over between flights.
void pidResetState();
