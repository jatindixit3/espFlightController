#include "settings.h"
#include <Preferences.h>
#include <string.h>

namespace {
constexpr uint32_t SETTINGS_MAGIC = 0x46433031; // "FC01"
constexpr uint16_t SETTINGS_VERSION = 2; // v2: added bidirDshotEnabled
constexpr const char* NVS_NAMESPACE = "fcsettings";
constexpr const char* NVS_KEY = "blob";

Settings g_settings;
Preferences g_prefs;

void applyDefaults(Settings& s) {
    memset(&s, 0, sizeof(Settings));
    s.magic = SETTINGS_MAGIC;
    s.version = SETTINGS_VERSION;

    // Conservative starting PIDs - safe-ish defaults for a 5" freestyle quad,
    // expect to need real tuning via the configurator, same as any fresh Betaflight board.
    for (int i = 0; i < 3; i++) {
        s.pid[i] = {45.0f, 80.0f, 30.0f, 60.0f};
    }
    // Yaw typically runs less D
    s.pid[2].D = 0.0f;

    s.dtermLowpassHz = 150.0f;
    s.gyroLowpassHz = 200.0f;
    s.rpmFilterEnabled = true;

    s.levelGainP = 6.0f;
    s.horizonTiltEffect = 75.0f;
    s.maxAngleDeg = 55.0f;

    for (int i = 0; i < 3; i++) {
        s.rates[i] = {1.0f, 0.7f, 0.0f};
    }

    s.motorIdlePercent = 5.5f;
    for (int i = 0; i < 4; i++) s.motorDirectionReversed[i] = false;
    s.motorInvertYaw = false;
    s.bidirDshotEnabled = true; // AM32 auto-detects the inverted protocol

    s.boardAlignRollDeg = 0.0f;
    s.boardAlignPitchDeg = 0.0f;
    s.boardAlignYawDeg = 0.0f;

    // Default aux mapping: AUX1 (SBUS ch4) = ARM, AUX2 = ANGLE, AUX3 = HORIZON, AUX4 = BLACKBOX.
    // All unassigned until the user maps them via the configurator's Modes tab -
    // ARM in particular defaults to *disabled* so a fresh board can never
    // accidentally arm before the pilot has configured a switch for it.
    for (int i = 0; i < MODE_COUNT; i++) {
        s.auxRanges[i] = {-1, 1700, 2100};
    }

    s.failsafeTimeoutMs = 200;
    s.failsafeDropThrottlePercent = 0.0f;

    s.batteryCellCountOverride = 0; // auto-detect
    s.batteryWarningVoltagePerCell = 3.5f;
    s.batteryCriticalVoltagePerCell = 3.3f;

    s.blackboxRateDivider = 2;
    s.motorPolePairs = 7;

    s.rxMinUs = 988;
    s.rxMaxUs = 2012;
    s.rxMidUs = 1500;
}
} // namespace

void settingsInit() {
    g_prefs.begin(NVS_NAMESPACE, false);

    size_t storedLen = g_prefs.getBytesLength(NVS_KEY);
    if (storedLen == sizeof(Settings)) {
        Settings loaded;
        g_prefs.getBytes(NVS_KEY, &loaded, sizeof(Settings));
        if (loaded.magic == SETTINGS_MAGIC && loaded.version == SETTINGS_VERSION) {
            g_settings = loaded;
            return;
        }
    }

    // Nothing valid stored (first boot, or version mismatch after a firmware update).
    applyDefaults(g_settings);
    g_prefs.putBytes(NVS_KEY, &g_settings, sizeof(Settings));
}

Settings& settingsGet() {
    return g_settings;
}

void settingsSave() {
    g_prefs.putBytes(NVS_KEY, &g_settings, sizeof(Settings));
}

void settingsResetToDefaults() {
    applyDefaults(g_settings);
    settingsSave();
}
