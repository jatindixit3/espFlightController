#pragma once
#include <stdint.h>

// ============================================================================
// Standard Betaflight-convention QuadX mixer.
// Motor order/table matches Betaflight's default so it maps the way you'd
// expect: M1 rear-right, M2 front-right, M3 rear-left, M4 front-left.
// Physical wire-to-position mismatches are resolved via the configurator's
// motor test tab (spin one at a time, props off, confirm position) - exactly
// like setting up any other Betaflight board, not something to get "correct"
// in firmware ahead of time.
// ============================================================================

struct MixerInput {
    float throttle;  // 0.0 - 1.0
    float rollPid;   // PID controller output
    float pitchPid;
    float yawPid;
};

void mixerInit();

// Computes final DShot throttle values for all 4 motors. When armed is false,
// always outputs all-zero (motor stop) regardless of input - the mixer itself
// enforces this, it does not trust callers to have zeroed the input.
void mixerCompute(const MixerInput& input, bool armed, uint16_t outThrottle[4]);
