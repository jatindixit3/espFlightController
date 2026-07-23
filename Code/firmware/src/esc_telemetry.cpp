#include "esc_telemetry.h"
#include "config.h"
#include <HardwareSerial.h>

namespace {
HardwareSerial TelemetrySerial(ESC_TELEMETRY_UART_NUM);

constexpr size_t PACKET_SIZE = 10;
constexpr uint32_t FRAME_GAP_MS = 3; // silence longer than this = new packet starts

uint8_t g_buf[PACKET_SIZE];
uint8_t g_bufIdx = 0;
uint32_t g_lastByteMs = 0;
int g_nextMotor = 0;

MotorTelemetry g_telemetry[4];

uint8_t crc8Update(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

bool crc8Check(const uint8_t* buf, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len - 1; i++) crc = crc8Update(crc, buf[i]);
    return crc == buf[len - 1];
}

void decodePacket(const uint8_t* p, int motorIndex) {
    MotorTelemetry& t = g_telemetry[motorIndex];
    t.temperatureC = p[0];
    t.voltage = ((uint16_t)(p[1] << 8 | p[2])) * 0.01f;
    t.current = ((uint16_t)(p[3] << 8 | p[4])) * 0.01f;
    t.consumptionMah = (uint16_t)(p[5] << 8 | p[6]);
    t.eRpm = (uint32_t)((uint16_t)(p[7] << 8 | p[8])) * 100u;
    t.lastUpdateMs = millis();
}
} // namespace

void escTelemetryInit() {
    TelemetrySerial.begin(ESC_TELEMETRY_BAUD, SERIAL_8N1, PIN_ESC_TELEMETRY_RX, -1, false);
    g_bufIdx = 0;
    g_lastByteMs = millis();
    g_nextMotor = 0;
    for (int i = 0; i < 4; i++) {
        g_telemetry[i] = {};
    }
}

void escTelemetryUpdate() {
    uint32_t now = millis();

    while (TelemetrySerial.available()) {
        if (g_bufIdx > 0 && (now - g_lastByteMs) > FRAME_GAP_MS) {
            // Silence gap mid-packet - the previous partial packet is junk, restart.
            g_bufIdx = 0;
        }

        g_buf[g_bufIdx++] = (uint8_t)TelemetrySerial.read();
        g_lastByteMs = now;
        now = millis();

        if (g_bufIdx == PACKET_SIZE) {
            if (crc8Check(g_buf, PACKET_SIZE)) {
                decodePacket(g_buf, g_nextMotor);
                g_nextMotor = (g_nextMotor + 1) % 4;
            }
            g_bufIdx = 0;
        }
    }

    // Mid-packet timeout even with no new bytes arriving (e.g. a byte was dropped).
    if (g_bufIdx > 0 && (now - g_lastByteMs) > FRAME_GAP_MS) {
        g_bufIdx = 0;
    }
}

const MotorTelemetry& escTelemetryGet(int motorIndex) {
    return g_telemetry[motorIndex];
}

float escTelemetryGetBatteryVoltage() {
    float sum = 0;
    int count = 0;
    uint32_t now = millis();
    for (int i = 0; i < 4; i++) {
        if (g_telemetry[i].lastUpdateMs != 0 && (now - g_telemetry[i].lastUpdateMs) < 1000) {
            sum += g_telemetry[i].voltage;
            count++;
        }
    }
    return count > 0 ? (sum / count) : 0.0f;
}

float escTelemetryGetTotalCurrent() {
    float sum = 0;
    uint32_t now = millis();
    for (int i = 0; i < 4; i++) {
        if (g_telemetry[i].lastUpdateMs != 0 && (now - g_telemetry[i].lastUpdateMs) < 1000) {
            sum += g_telemetry[i].current;
        }
    }
    return sum;
}
