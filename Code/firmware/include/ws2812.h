#pragma once
#include <stdint.h>

// ============================================================================
// Minimal WS2812 driver over SPI instead of RMT.
//
// Why not RMT (or the Adafruit NeoPixel library, which uses RMT on ESP32):
// the S3 has exactly 4 TX-capable RMT channels (0-3) and all four now belong
// to the motors - with bidirectional DShot the RX-capable channels (4-7) are
// taken too. Any RMT-based LED driver would fight the motor outputs for a
// channel; depending on init order that contention could have silently killed
// motor 1's output. SPI sidesteps the whole class of problem: each WS2812 bit
// is encoded as 4 SPI bits at 3.2MHz ('1' -> 1110, '0' -> 1000), giving
// in-spec pulse widths with DMA-fed hardware timing and zero RMT usage.
//
// Only drives all LEDs to a single color (all this FC's status patterns need).
// ============================================================================

void ws2812Init();

// Applies the fixed global brightness and pushes the strip. Blocking for the
// SPI transfer (~0.4ms for 8 LEDs at 3.2MHz) - call from the indicators task,
// never the flight loop.
void ws2812SetAll(uint8_t r, uint8_t g, uint8_t b);
