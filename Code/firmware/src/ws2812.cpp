#include "ws2812.h"
#include "config.h"
#include <SPI.h>
#include <string.h>

namespace {

// 3.2MHz -> 312.5ns per SPI bit; 4 SPI bits per WS2812 bit (1.25us period).
// '0' = 1000 (high 312ns, spec 220-380ns), '1' = 1110 (high 937ns, tolerated
// by WS2812B's wide T1H window). Trailing zero bytes hold the line low for
// >80us, which is the latch/reset condition.
constexpr uint32_t SPI_HZ = 3200000;
constexpr int RESET_BYTES = 48; // 48 bytes * 8 bits * 312.5ns = 120us of low

// SCK/MISO must be attached to real pins but are unused - GPIO13/14 are on
// the S3 module but not broken out on the XIAO, and free of flash/PSRAM use.
constexpr int PIN_SCK_DUMMY = 13;
constexpr int PIN_MISO_DUMMY = 14;

constexpr uint8_t BRIGHTNESS = 80; // 0-255 global scale (matches the old setBrightness(80))

SPIClass g_spi(HSPI);
uint8_t g_buf[LED_COUNT * 3 * 4 + RESET_BYTES];

// Expands one color byte (MSB first) into 4 SPI bytes, 2 WS2812 bits each.
void encodeByte(uint8_t value, uint8_t* out) {
    for (int i = 0; i < 4; i++) {
        uint8_t twoBits = (value >> (6 - 2 * i)) & 0x3;
        uint8_t hiNibble = (twoBits & 0x2) ? 0xE0 : 0x80;
        uint8_t loNibble = (twoBits & 0x1) ? 0x0E : 0x08;
        out[i] = hiNibble | loNibble;
    }
}

} // namespace

void ws2812Init() {
    g_spi.begin(PIN_SCK_DUMMY, PIN_MISO_DUMMY, PIN_LED_DATA, -1);
    ws2812SetAll(0, 0, 0);
}

void ws2812SetAll(uint8_t r, uint8_t g, uint8_t b) {
    r = (uint8_t)((uint16_t)r * BRIGHTNESS / 255);
    g = (uint8_t)((uint16_t)g * BRIGHTNESS / 255);
    b = (uint8_t)((uint16_t)b * BRIGHTNESS / 255);

    uint8_t* p = g_buf;
    for (int led = 0; led < LED_COUNT; led++) {
        encodeByte(g, p); p += 4; // WS2812 wire order is GRB
        encodeByte(r, p); p += 4;
        encodeByte(b, p); p += 4;
    }
    memset(p, 0, RESET_BYTES);

    g_spi.beginTransaction(SPISettings(SPI_HZ, SPI_MSBFIRST, SPI_MODE0));
    g_spi.writeBytes(g_buf, sizeof(g_buf));
    g_spi.endTransaction();
}
