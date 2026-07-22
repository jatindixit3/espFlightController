#pragma once
#include <stdint.h>

// ============================================================================
// DShot300 motor output via the ESP32 RMT peripheral (one channel per motor).
//
// Design note: your ESCs support both bidirectional DShot (eRPM over the same
// signal wire) and a separate telemetry wire. This firmware deliberately uses
// ONLY the separate telemetry wire (see esc_telemetry.h) for RPM/voltage/
// current, and keeps this DShot output plain and unidirectional.
//
// Why: bidirectional DShot's return path is a GCR-encoded bit-banged decode
// that is genuinely difficult to get right and effectively impossible to
// verify without a logic analyzer and real ESCs on a bench - exactly the kind
// of "looks plausible, subtly wrong" code that has no business anywhere near
// a spinning propeller. The telemetry wire gives the same eRPM data (plus
// voltage/current/temp) over a simple, well-defined UART protocol at a small
// fraction of the risk. Motor throttle output (this module) never depends on
// telemetry working at all, in either direction.
// ============================================================================

void dshotInit();

// Commands all motors in one shot. Each value is 0 (motor stop / disarmed) or
// DSHOT_MIN_THROTTLE..DSHOT_MAX_THROTTLE (48-2047). Called once per flight
// loop iteration - non-blocking (each ~53us DShot300 frame safely completes
// well within the 1ms loop period before the next call).
void dshotWriteThrottles(const uint16_t throttle[4]);

// Immediately commands all motors to the stop value. Used by the arming/
// failsafe logic as the hard, always-available "motors off now" path.
void dshotStopAll();
