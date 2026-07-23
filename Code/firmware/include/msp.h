#pragma once
#include <stdint.h>

// ============================================================================
// MSP-style binary protocol server over native USB CDC (the same port used
// for flashing). This is a custom, MSPv1-*framed* protocol (same $M< / $M>
// header/checksum shape as real MSP) but with our own command set - it talks
// to Code/configurator/js/msp-client.js in this repo, not the real Betaflight
// Configurator. See Documentation/PROTOCOL.md for the exact wire format.
// ============================================================================

void mspInit();

// Drains incoming USB CDC bytes, parses complete frames, dispatches commands,
// writes responses. Call from a low/medium-priority task (MSP_TASK_HZ is plenty).
void mspUpdate();

// True (with outThrottle filled) if a motor-test command arrived within the
// last second AND the aircraft is currently disarmed. The flight loop uses
// this to bypass the mixer/PID for bench motor testing - deliberately
// unavailable while armed, so a stray test command mid-flight can't do
// anything.
bool mspGetMotorTestOverride(uint16_t outThrottle[4]);
