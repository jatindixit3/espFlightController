#pragma once
#include <stdint.h>

// ============================================================================
// Persistent configuration - the ESP32 NVS-backed equivalent of Betaflight's
// EEPROM config. One blob, versioned, loaded at boot and saved on demand
// (never written mid-flight - flash writes are slow and would stall the loop).
// ============================================================================

enum ModeId : uint8_t {
    MODE_ARM = 0,
    MODE_ANGLE,
    MODE_HORIZON,
    MODE_BLACKBOX,
    MODE_COUNT
};

struct AuxRange {
    int8_t channel;        // AUX channel index (0 = AUX1 = SBUS channel 4), -1 = unassigned
    uint16_t rangeStartUs;  // inclusive, in RC microsecond units (988-2012 typical)
    uint16_t rangeEndUs;    // exclusive
};

struct AxisPid {
    float P;
    float I;
    float D;
    float FF;
};

struct RateProfile {
    float rcRate;
    float superRate;
    float expo;
};

struct Settings {
    uint32_t magic;         // validity marker
    uint16_t version;       // bump when struct layout changes -> triggers defaults reload

    // --- PID ---
    AxisPid pid[3];         // 0=roll, 1=pitch, 2=yaw
    float dtermLowpassHz;
    float gyroLowpassHz;
    bool rpmFilterEnabled;

    // --- Angle/Horizon leveling ---
    float levelGainP;            // self-level aggressiveness (angle mode P gain)
    float horizonTiltEffect;     // 0-100: how quickly horizon hands off from level to acro as stick moves off-center
    float maxAngleDeg;           // angle-mode tilt limit

    // --- Rates ---
    RateProfile rates[3];   // 0=roll, 1=pitch, 2=yaw

    // --- Mixer / motor protocol ---
    float motorIdlePercent;    // 0-100, spun-up idle to avoid desync
    bool motorDirectionReversed[4];
    bool motorInvertYaw;       // swap yaw sign, e.g. props-out builds
    bool bidirDshotEnabled;    // inverted DShot300 + per-motor eRPM readback; needs save+reboot to change

    // --- Board alignment (mounting orientation) ---
    float boardAlignRollDeg;
    float boardAlignPitchDeg;
    float boardAlignYawDeg;

    // --- Modes ---
    AuxRange auxRanges[MODE_COUNT];

    // --- Failsafe ---
    uint32_t failsafeTimeoutMs;
    float failsafeDropThrottlePercent; // throttle to command during failsafe hold stage

    // --- Battery ---
    int8_t batteryCellCountOverride; // 0 = auto-detect from ESC telemetry voltage
    float batteryWarningVoltagePerCell;
    float batteryCriticalVoltagePerCell;

    // --- Blackbox ---
    uint8_t blackboxRateDivider; // log every Nth loop iteration

    // --- Motor characteristics (for RPM notch frequency calc) ---
    uint8_t motorPolePairs; // e.g. 7 for a common 12N14P 5" freestyle motor

    // --- Rx ---
    uint16_t rxMinUs;
    uint16_t rxMaxUs;
    uint16_t rxMidUs;
};

// Loads settings from NVS, or writes+returns defaults if none/invalid stored.
void settingsInit();

Settings& settingsGet();

// Persists the current in-memory settings to NVS. Call only outside the flight
// loop (e.g. from the MSP task after a `save` command) - flash writes can take
// tens of milliseconds.
void settingsSave();

void settingsResetToDefaults();
