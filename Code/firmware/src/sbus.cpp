#include "sbus.h"
#include "config.h"
#include <HardwareSerial.h>
#include <math.h>

namespace {
HardwareSerial SbusSerial(SBUS_UART_NUM);

constexpr uint8_t SBUS_FRAME_SIZE = 25;
constexpr uint8_t SBUS_START_BYTE = 0x0F;

uint8_t g_buf[SBUS_FRAME_SIZE];
uint8_t g_bufIdx = 0;

SbusFrame g_frame;
uint32_t g_lastValidFrameMs = 0;

bool isValidEndByte(uint8_t b) {
    // Standard SBUS end byte is 0x00; some receiver variants use SBUS2 slot
    // markers (0x04/0x14/0x24/0x34) in the same position - accept all of them.
    return b == 0x00 || b == 0x04 || b == 0x14 || b == 0x24 || b == 0x34;
}

void decodeFrame(const uint8_t* f) {
    // f[0]=start, f[1..22]=packed 16x11-bit channels, f[23]=flags, f[24]=end
    const uint8_t* p = &f[1];
    uint32_t bitBuffer = 0;
    int bitsInBuffer = 0;
    int byteIdx = 0;

    for (int ch = 0; ch < SBUS_CHANNEL_COUNT; ch++) {
        while (bitsInBuffer < 11) {
            bitBuffer |= ((uint32_t)p[byteIdx++]) << bitsInBuffer;
            bitsInBuffer += 8;
        }
        g_frame.channels[ch] = bitBuffer & 0x07FF;
        bitBuffer >>= 11;
        bitsInBuffer -= 11;
    }

    uint8_t flags = f[23];
    g_frame.digitalCh17 = flags & 0x01;
    g_frame.digitalCh18 = flags & 0x02;
    g_frame.frameLost   = flags & 0x04;
    g_frame.failsafe    = flags & 0x08;
}
} // namespace

void sbusInit() {
    // invert=true: SBUS is an inverted UART signal - the ESP32 hardware inverts
    // it for us, no external inverter transistor needed. RX only, TX pin unused.
    SbusSerial.begin(SBUS_BAUD, SERIAL_8E2, PIN_SBUS_RX, -1, true);
    g_lastValidFrameMs = 0;
    g_bufIdx = 0;
}

void sbusUpdate() {
    while (SbusSerial.available()) {
        uint8_t b = (uint8_t)SbusSerial.read();

        if (g_bufIdx == 0 && b != SBUS_START_BYTE) {
            continue; // resync: discard bytes until we see a start byte
        }

        g_buf[g_bufIdx++] = b;

        if (g_bufIdx == SBUS_FRAME_SIZE) {
            if (isValidEndByte(g_buf[SBUS_FRAME_SIZE - 1])) {
                decodeFrame(g_buf);
                g_lastValidFrameMs = millis();
            }
            g_bufIdx = 0;
        }
    }
}

bool sbusIsLinkHealthy() {
    if (g_lastValidFrameMs == 0) return false;
    return (millis() - g_lastValidFrameMs) < SBUS_FAILSAFE_TIMEOUT_MS;
}

const SbusFrame& sbusGetFrame() {
    return g_frame;
}

uint16_t sbusChannelToUs(uint16_t rawChannelValue) {
    return (uint16_t)lroundf(rawChannelValue * 0.625f + 880.0f);
}
