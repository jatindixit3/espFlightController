#include "mixer.h"
#include "config.h"
#include "settings.h"

namespace {
struct MixTableEntry { float roll, pitch, yaw; };

// Betaflight default QuadX table: {roll, pitch, yaw} per motor.
constexpr MixTableEntry MIX_TABLE[4] = {
    {-1.0f, +1.0f, -1.0f}, // M1 rear right
    {-1.0f, -1.0f, +1.0f}, // M2 front right
    {+1.0f, +1.0f, +1.0f}, // M3 rear left
    {+1.0f, -1.0f, -1.0f}, // M4 front left
};
} // namespace

void mixerInit() {}

void mixerCompute(const MixerInput& input, bool armed, uint16_t outThrottle[4]) {
    if (!armed) {
        for (int i = 0; i < 4; i++) outThrottle[i] = 0;
        return;
    }

    Settings& s = settingsGet();
    float yaw = input.yawPid * (s.motorInvertYaw ? -1.0f : 1.0f);

    float raw[4];
    float maxOut = 1.0f;
    for (int i = 0; i < 4; i++) {
        raw[i] = input.throttle
               + MIX_TABLE[i].roll * input.rollPid
               + MIX_TABLE[i].pitch * input.pitchPid
               + MIX_TABLE[i].yaw * yaw;
        if (raw[i] > maxOut) maxOut = raw[i];
    }

    // If any motor wants more than full output, scale every motor down by the
    // same factor rather than clipping only the saturated one - preserves the
    // commanded attitude correction ratio instead of distorting it under load.
    float scale = 1.0f / maxOut;

    float idle = s.motorIdlePercent / 100.0f;
    for (int i = 0; i < 4; i++) {
        float v = raw[i] * scale;
        if (v < idle) v = idle;
        if (v > 1.0f) v = 1.0f;
        outThrottle[i] = (uint16_t)(DSHOT_MIN_THROTTLE + v * (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE));
    }

    // NOTE: settings.motorDirectionReversed is currently unused - set spin
    // direction via your AM32 configurator/passthrough (or swap two motor
    // wires physically), not from this firmware. Reserved for a future
    // proper implementation using DShot's boot-time direction-set command
    // sequence, which needs careful, hardware-verified timing to do safely.
}
