#include "flight_state.h"
#include "sbus.h"
#include "imu.h"
#include "settings.h"
#include "config.h"
#include <math.h>

namespace {
FlightState g_state;
constexpr float STICK_DEADBAND = 0.02f;

// Standard AETR channel order: 0=roll(A), 1=pitch(E), 2=throttle(T), 3=yaw(R), 4+=AUX1..
constexpr int CH_ROLL = 0;
constexpr int CH_PITCH = 1;
constexpr int CH_THROTTLE = 2;
constexpr int CH_YAW = 3;
constexpr int CH_AUX1 = 4;

float normalizeStick(uint16_t us, uint16_t minUs, uint16_t midUs, uint16_t maxUs) {
    float v;
    if (us >= midUs) {
        v = (float)(us - midUs) / (float)(maxUs - midUs);
    } else {
        v = (float)(us - midUs) / (float)(midUs - minUs);
    }
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    if (fabsf(v) < STICK_DEADBAND) v = 0.0f;
    return v;
}

float normalizeThrottle(uint16_t us, uint16_t minUs, uint16_t maxUs) {
    float v = (float)(us - minUs) / (float)(maxUs - minUs);
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return v;
}

bool isAuxModeActive(ModeId mode) {
    Settings& s = settingsGet();
    const AuxRange& r = s.auxRanges[mode];
    if (r.channel < 0) return false;
    int chIndex = CH_AUX1 + r.channel;
    if (chIndex >= SBUS_CHANNEL_COUNT) return false;
    uint16_t us = sbusChannelToUs(sbusGetFrame().channels[chIndex]);
    return us >= r.rangeStartUs && us < r.rangeEndUs;
}

} // namespace

void flightStateInit() {
    g_state.armed = false;
    g_state.failsafeActive = false;
    g_state.mode = FLIGHT_MODE_ACRO;
    g_state.blackboxEnabled = false;
    for (int i = 0; i < 3; i++) g_state.rcCommand[i] = 0;
    g_state.throttle = 0;
}

void flightStateUpdate() {
    Settings& s = settingsGet();
    const SbusFrame& frame = sbusGetFrame();
    bool linkHealthy = sbusIsLinkHealthy() && !frame.failsafe && !frame.frameLost;

    g_state.failsafeActive = !linkHealthy;

    if (linkHealthy) {
        uint16_t rollUs = sbusChannelToUs(frame.channels[CH_ROLL]);
        uint16_t pitchUs = sbusChannelToUs(frame.channels[CH_PITCH]);
        uint16_t throttleUs = sbusChannelToUs(frame.channels[CH_THROTTLE]);
        uint16_t yawUs = sbusChannelToUs(frame.channels[CH_YAW]);

        g_state.rcCommand[0] = normalizeStick(rollUs, s.rxMinUs, s.rxMidUs, s.rxMaxUs);
        g_state.rcCommand[1] = normalizeStick(pitchUs, s.rxMinUs, s.rxMidUs, s.rxMaxUs);
        g_state.rcCommand[2] = normalizeStick(yawUs, s.rxMinUs, s.rxMidUs, s.rxMaxUs);
        g_state.throttle = normalizeThrottle(throttleUs, s.rxMinUs, s.rxMaxUs);
    }
    // On unhealthy link, deliberately hold the last known stick/throttle
    // values - it's harmless, because the failsafe disarm below forces the
    // mixer to zero every motor whenever armed is false, regardless of what
    // rcCommand/throttle contain.

    if (isAuxModeActive(MODE_ANGLE)) {
        g_state.mode = FLIGHT_MODE_ANGLE;
    } else if (isAuxModeActive(MODE_HORIZON)) {
        g_state.mode = FLIGHT_MODE_HORIZON;
    } else {
        g_state.mode = FLIGHT_MODE_ACRO;
    }

    g_state.blackboxEnabled = isAuxModeActive(MODE_BLACKBOX);

    bool armSwitchActive = isAuxModeActive(MODE_ARM);

    if (g_state.armed) {
        // Staying armed requires the link to be healthy and the switch to
        // still be in the arm range - anything else disarms immediately,
        // matching how any real FC treats an arm-switch flip mid-flight.
        if (!linkHealthy || !armSwitchActive) {
            g_state.armed = false;
        }
    } else {
        bool throttleLow = g_state.throttle < 0.05f;
        if (armSwitchActive && throttleLow && linkHealthy && imuIsCalibrated()) {
            g_state.armed = true;
            pidResetState();
        }
    }
}

const FlightState& flightStateGet() {
    return g_state;
}
