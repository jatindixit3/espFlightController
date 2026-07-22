#pragma once
#include <stdint.h>

// ============================================================================
// SBUS receiver input. Inverted UART, 100000 baud, 8E2, 25-byte frames,
// 16 proportional channels (11-bit) + 2 digital + frame-lost/failsafe flags.
// RX-only - we never transmit anything back to the receiver.
// ============================================================================

constexpr int SBUS_CHANNEL_COUNT = 16;

struct SbusFrame {
    uint16_t channels[SBUS_CHANNEL_COUNT]; // raw 11-bit values, 0-2047
    bool digitalCh17;
    bool digitalCh18;
    bool frameLost;     // this frame's FRAME_LOST flag
    bool failsafe;      // this frame's FAILSAFE flag
};

void sbusInit();

// Call frequently (every flight loop iteration is fine) - drains the UART FIFO
// and updates the latest decoded frame when a complete one arrives.
void sbusUpdate();

// True if a valid frame has been decoded within SBUS_FAILSAFE_TIMEOUT_MS.
bool sbusIsLinkHealthy();

const SbusFrame& sbusGetFrame();

// Converts a raw 11-bit SBUS channel value to the standard 988-2012us RC range.
uint16_t sbusChannelToUs(uint16_t rawChannelValue);
