#pragma once
#include <stdint.h>

// ============================================================================
// DShot300 motor output via the ESP32-S3 RMT peripheral, with optional
// bidirectional DShot (settings.bidirDshotEnabled, default ON).
//
// Bidirectional mode: frames are sent inverted (line idles HIGH) with the
// complemented checksum that tells the ESC to answer, and after each frame
// the pin is switched to input so the ESC can send back a 21-bit GCR-encoded
// eRPM response at 5/4 the TX bitrate. The S3's RMT has 4 TX-only channels
// (0-3, one per motor) and 4 RX-only channels (4-7) - each motor gets a TX+RX
// pair attached to the same GPIO through the GPIO matrix.
//
// The separate ESC telemetry wire (esc_telemetry.h) stays wired and parsed:
// it is the only source of voltage/current/temperature, and the RPM filter
// falls back to its (slower) eRPM whenever bidir responses go stale - so a
// decode problem in this module degrades filter quality, never motor control.
//
// Honest caveat: the GCR response decode is CRC-checked (garbage is dropped,
// never acted on) but has not been verified against real ESC hardware - see
// Documentation/BENCH_TEST.md step 7 for how to confirm it on the bench, and the
// configurator's Configuration tab for the toggle (save + reboot to apply).
// ============================================================================

struct DshotTelemetry {
    uint32_t eRpm;         // electrical RPM from the latest valid bidir response
    uint32_t lastUpdateMs; // millis() of that response; 0 = never received
};

void dshotInit();

// Commands all motors in one shot. Each value is 0 (motor stop / disarmed) or
// DSHOT_MIN_THROTTLE..DSHOT_MAX_THROTTLE (48-2047). Called once per flight
// loop iteration. In bidir mode this also drains/decodes the previous cycle's
// eRPM responses and re-arms capture (~120us total, well inside the 1ms loop).
void dshotWriteThrottles(const uint16_t throttle[4]);

// Immediately commands all motors to the stop value. Used by the arming/
// failsafe logic as the hard, always-available "motors off now" path.
void dshotStopAll();

// Latest bidir eRPM for a motor. lastUpdateMs==0 or stale means no usable
// bidir data - callers must fall back to escTelemetryGet().
const DshotTelemetry& dshotGetTelemetry(int motorIndex);

// Requests a persistent spin-direction change for one PHYSICAL motor output.
// Sends the real DShot SPIN_DIRECTION_NORMAL/REVERSED command followed by
// SAVE_SETTINGS, spread over the next ~24 flight-loop frames; the ESC stores
// it. Thread-safe to call from the comms task. The change is only started if
// all motors are stopped (i.e. disarmed) when the flight loop picks it up, so
// direction can never flip on a spinning motor. Call only while disarmed.
void dshotRequestDirection(int motorIndex, bool reversed);

// True while a direction request is pending or a sequence is running. Motor
// test and arming should be suppressed while this is true.
bool dshotDirectionBusy();
