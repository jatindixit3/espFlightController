#pragma once
#include <stdint.h>
#include "pid.h"

// ============================================================================
// Arming, failsafe, and AUX-range flight mode evaluation.
//
// Safety invariants enforced here (not just "expected" of callers):
//  - Cannot arm unless: gyro calibrated, SBUS link healthy, throttle is at
//    minimum, and the configured ARM aux range is active.
//  - Losing SBUS link while armed forces an immediate failsafe disarm - motors
//    cut, not a staged descent (simplest-safe default for a freestyle quad
//    with no GPS/baro to do anything smarter with).
//  - Moving the arm switch back to its disarm range disarms immediately
//    regardless of throttle position, same as any real FC.
// ============================================================================

struct FlightState {
    bool armed;
    bool failsafeActive;
    FlightMode mode;
    bool blackboxEnabled;
    float rcCommand[3]; // roll, pitch, yaw: -1..1, deadband applied
    float throttle;      // 0..1
};

void flightStateInit();

// Call once per flight loop iteration, after sbusUpdate().
void flightStateUpdate();

const FlightState& flightStateGet();
