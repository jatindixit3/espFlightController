#include "msp.h"
#include "config.h"
#include "settings.h"
#include "imu.h"
#include "sbus.h"
#include "esc_telemetry.h"
#include "blackbox.h"
#include "flight_state.h"
#include "pid.h"
#include <Arduino.h>
#include <string.h>

namespace {

enum : uint8_t {
    MSP_IDENTIFY = 100,
    MSP_STATUS = 101,
    MSP_RAW_IMU = 102,
    MSP_ATTITUDE = 103,
    MSP_RC = 104,
    MSP_MOTOR = 105,
    MSP_PID = 106,
    MSP_SET_PID = 107,
    MSP_RATES = 108,
    MSP_SET_RATES = 109,
    MSP_MODES = 110,
    MSP_SET_MODES = 111,
    MSP_MISC = 112,
    MSP_SET_MISC = 113,
    MSP_CALIBRATE_GYRO = 114,
    MSP_MOTOR_TEST = 115,
    MSP_ESC_TELEMETRY = 116,
    MSP_BLACKBOX_INFO = 117,
    MSP_BLACKBOX_READ = 118,
    MSP_BLACKBOX_ERASE = 119,
    MSP_SAVE_SETTINGS = 120,
    MSP_RESET_DEFAULTS = 121,
    MSP_REBOOT = 122,
};

constexpr uint8_t FRAME_MAX_PAYLOAD = 250;

enum class ParseState { IDLE, GOT_DOLLAR, GOT_M, GOT_DIR, GOT_SIZE, GOT_CMD, GOT_PAYLOAD };
ParseState g_state = ParseState::IDLE;
uint8_t g_expectedSize = 0;
uint8_t g_cmd = 0;
uint8_t g_payload[FRAME_MAX_PAYLOAD];
uint8_t g_payloadIdx = 0;
uint8_t g_checksum = 0;

uint16_t g_motorTestThrottle[4] = {0, 0, 0, 0};
uint32_t g_motorTestLastMs = 0;

struct Writer {
    uint8_t buf[FRAME_MAX_PAYLOAD];
    uint8_t len = 0;
    void u8(uint8_t v) { buf[len++] = v; }
    void u16(uint16_t v) { memcpy(&buf[len], &v, 2); len += 2; }
    void u32(uint32_t v) { memcpy(&buf[len], &v, 4); len += 4; }
    void i8(int8_t v) { buf[len++] = (uint8_t)v; }
    void f32(float v) { memcpy(&buf[len], &v, 4); len += 4; }
};

struct Reader {
    const uint8_t* buf;
    uint8_t pos = 0;
    uint8_t u8() { return buf[pos++]; }
    uint16_t u16() { uint16_t v; memcpy(&v, &buf[pos], 2); pos += 2; return v; }
    uint32_t u32() { uint32_t v; memcpy(&v, &buf[pos], 4); pos += 4; return v; }
    int8_t i8() { return (int8_t)buf[pos++]; }
    float f32() { float v; memcpy(&v, &buf[pos], 4); pos += 4; return v; }
};

void sendFrame(uint8_t cmd, const uint8_t* payload, uint8_t len) {
    uint8_t checksum = (uint8_t)(len ^ cmd);
    for (uint8_t i = 0; i < len; i++) checksum ^= payload[i];
    Serial.write('$'); Serial.write('M'); Serial.write('>');
    Serial.write(len);
    Serial.write(cmd);
    if (len) Serial.write(payload, len);
    Serial.write(checksum);
}

void handleIdentify() {
    Writer w;
    const char* name = "ESP32FC-1.0";
    for (int i = 0; name[i] != '\0'; i++) w.u8((uint8_t)name[i]);
    sendFrame(MSP_IDENTIFY, w.buf, w.len);
}

void handleStatus() {
    Writer w;
    const FlightState& fs = flightStateGet();
    w.u8(fs.armed ? 1 : 0);
    w.u8(fs.failsafeActive ? 1 : 0);
    w.u8((uint8_t)fs.mode);
    w.u8(imuIsCalibrated() ? 1 : 0);
    w.u16((uint16_t)(escTelemetryGetBatteryVoltage() * 100.0f));
    w.u16((uint16_t)(escTelemetryGetTotalCurrent() * 100.0f));
    sendFrame(MSP_STATUS, w.buf, w.len);
}

void handleRawImu() {
    Writer w;
    const ImuSample& s = imuGetSample();
    for (int i = 0; i < 3; i++) w.f32(s.gyroDegS[i]);
    for (int i = 0; i < 3; i++) w.f32(s.accelG[i]);
    sendFrame(MSP_RAW_IMU, w.buf, w.len);
}

void handleAttitude() {
    Writer w;
    const ImuSample& s = imuGetSample();
    for (int i = 0; i < 3; i++) w.f32(s.attitudeDeg[i]);
    sendFrame(MSP_ATTITUDE, w.buf, w.len);
}

void handleRc() {
    Writer w;
    const SbusFrame& f = sbusGetFrame();
    for (int i = 0; i < SBUS_CHANNEL_COUNT; i++) w.u16(sbusChannelToUs(f.channels[i]));
    sendFrame(MSP_RC, w.buf, w.len);
}

void handlePid() {
    Writer w;
    Settings& s = settingsGet();
    for (int i = 0; i < 3; i++) {
        w.f32(s.pid[i].P); w.f32(s.pid[i].I); w.f32(s.pid[i].D); w.f32(s.pid[i].FF);
    }
    w.f32(s.dtermLowpassHz);
    w.f32(s.gyroLowpassHz);
    w.u8(s.rpmFilterEnabled ? 1 : 0);
    w.f32(s.levelGainP);
    w.f32(s.horizonTiltEffect);
    w.f32(s.maxAngleDeg);
    sendFrame(MSP_PID, w.buf, w.len);
}

void handleSetPid(Reader& r) {
    Settings& s = settingsGet();
    for (int i = 0; i < 3; i++) {
        s.pid[i].P = r.f32(); s.pid[i].I = r.f32(); s.pid[i].D = r.f32(); s.pid[i].FF = r.f32();
    }
    s.dtermLowpassHz = r.f32();
    s.gyroLowpassHz = r.f32();
    s.rpmFilterEnabled = r.u8() != 0;
    s.levelGainP = r.f32();
    s.horizonTiltEffect = r.f32();
    s.maxAngleDeg = r.f32();
    sendFrame(MSP_SET_PID, nullptr, 0);
}

void handleRates() {
    Writer w;
    Settings& s = settingsGet();
    for (int i = 0; i < 3; i++) { w.f32(s.rates[i].rcRate); w.f32(s.rates[i].superRate); w.f32(s.rates[i].expo); }
    sendFrame(MSP_RATES, w.buf, w.len);
}

void handleSetRates(Reader& r) {
    Settings& s = settingsGet();
    for (int i = 0; i < 3; i++) { s.rates[i].rcRate = r.f32(); s.rates[i].superRate = r.f32(); s.rates[i].expo = r.f32(); }
    sendFrame(MSP_SET_RATES, nullptr, 0);
}

void handleModes() {
    Writer w;
    Settings& s = settingsGet();
    for (int i = 0; i < MODE_COUNT; i++) {
        w.i8(s.auxRanges[i].channel);
        w.u16(s.auxRanges[i].rangeStartUs);
        w.u16(s.auxRanges[i].rangeEndUs);
    }
    sendFrame(MSP_MODES, w.buf, w.len);
}

void handleSetModes(Reader& r) {
    Settings& s = settingsGet();
    for (int i = 0; i < MODE_COUNT; i++) {
        s.auxRanges[i].channel = r.i8();
        s.auxRanges[i].rangeStartUs = r.u16();
        s.auxRanges[i].rangeEndUs = r.u16();
    }
    sendFrame(MSP_SET_MODES, nullptr, 0);
}

void handleMisc() {
    Writer w;
    Settings& s = settingsGet();
    w.u16(s.rxMinUs); w.u16(s.rxMidUs); w.u16(s.rxMaxUs);
    w.f32(s.motorIdlePercent);
    w.i8(s.batteryCellCountOverride);
    w.f32(s.batteryWarningVoltagePerCell);
    w.f32(s.batteryCriticalVoltagePerCell);
    w.u8(s.blackboxRateDivider);
    w.u8(s.motorPolePairs);
    w.u32(s.failsafeTimeoutMs);
    w.f32(s.boardAlignRollDeg);
    w.f32(s.boardAlignPitchDeg);
    w.f32(s.boardAlignYawDeg);
    sendFrame(MSP_MISC, w.buf, w.len);
}

void handleSetMisc(Reader& r) {
    Settings& s = settingsGet();
    s.rxMinUs = r.u16(); s.rxMidUs = r.u16(); s.rxMaxUs = r.u16();
    s.motorIdlePercent = r.f32();
    s.batteryCellCountOverride = r.i8();
    s.batteryWarningVoltagePerCell = r.f32();
    s.batteryCriticalVoltagePerCell = r.f32();
    s.blackboxRateDivider = r.u8();
    s.motorPolePairs = r.u8();
    s.failsafeTimeoutMs = r.u32();
    s.boardAlignRollDeg = r.f32();
    s.boardAlignPitchDeg = r.f32();
    s.boardAlignYawDeg = r.f32();
    // Apply immediately so the change is visible without a reboot.
    imuSetBoardAlignment(s.boardAlignRollDeg, s.boardAlignPitchDeg, s.boardAlignYawDeg);
    sendFrame(MSP_SET_MISC, nullptr, 0);
}

void handleCalibrateGyro() {
    bool ok = imuCalibrateGyro();
    Writer w;
    w.u8(ok ? 1 : 0);
    sendFrame(MSP_CALIBRATE_GYRO, w.buf, w.len);
}

void handleMotorTest(Reader& r) {
    for (int i = 0; i < 4; i++) g_motorTestThrottle[i] = r.u16();
    g_motorTestLastMs = millis();
    sendFrame(MSP_MOTOR_TEST, nullptr, 0);
}

void handleEscTelemetry() {
    Writer w;
    for (int i = 0; i < 4; i++) {
        const MotorTelemetry& t = escTelemetryGet(i);
        w.u8(t.temperatureC);
        w.f32(t.voltage);
        w.f32(t.current);
        w.u16(t.consumptionMah);
        w.u32(t.eRpm);
        w.u32(t.lastUpdateMs);
    }
    sendFrame(MSP_ESC_TELEMETRY, w.buf, w.len);
}

void handleBlackboxInfo() {
    Writer w;
    w.u32(blackboxGetWriteOffset());
    w.u32(blackboxGetPartitionSize());
    sendFrame(MSP_BLACKBOX_INFO, w.buf, w.len);
}

void handleBlackboxRead(Reader& r) {
    uint32_t offset = r.u32();
    uint16_t length = r.u16();
    if (length > FRAME_MAX_PAYLOAD) length = FRAME_MAX_PAYLOAD;
    Writer w;
    if (blackboxReadRaw(offset, w.buf, length)) {
        w.len = (uint8_t)length;
    } else {
        w.len = 0;
    }
    sendFrame(MSP_BLACKBOX_READ, w.buf, w.len);
}

void handleBlackboxErase() {
    blackboxEraseAll();
    sendFrame(MSP_BLACKBOX_ERASE, nullptr, 0);
}

void handleSaveSettings() {
    settingsSave();
    sendFrame(MSP_SAVE_SETTINGS, nullptr, 0);
}

void handleResetDefaults() {
    settingsResetToDefaults();
    sendFrame(MSP_RESET_DEFAULTS, nullptr, 0);
}

void handleReboot() {
    sendFrame(MSP_REBOOT, nullptr, 0);
    Serial.flush();
    delay(50);
    ESP.restart();
}

void dispatch(uint8_t cmd, const uint8_t* payload, uint8_t len) {
    Reader r{payload, 0};
    (void)len;
    switch (cmd) {
        case MSP_IDENTIFY: handleIdentify(); break;
        case MSP_STATUS: handleStatus(); break;
        case MSP_RAW_IMU: handleRawImu(); break;
        case MSP_ATTITUDE: handleAttitude(); break;
        case MSP_RC: handleRc(); break;
        case MSP_PID: handlePid(); break;
        case MSP_SET_PID: handleSetPid(r); break;
        case MSP_RATES: handleRates(); break;
        case MSP_SET_RATES: handleSetRates(r); break;
        case MSP_MODES: handleModes(); break;
        case MSP_SET_MODES: handleSetModes(r); break;
        case MSP_MISC: handleMisc(); break;
        case MSP_SET_MISC: handleSetMisc(r); break;
        case MSP_CALIBRATE_GYRO: handleCalibrateGyro(); break;
        case MSP_MOTOR_TEST: handleMotorTest(r); break;
        case MSP_ESC_TELEMETRY: handleEscTelemetry(); break;
        case MSP_BLACKBOX_INFO: handleBlackboxInfo(); break;
        case MSP_BLACKBOX_READ: handleBlackboxRead(r); break;
        case MSP_BLACKBOX_ERASE: handleBlackboxErase(); break;
        case MSP_SAVE_SETTINGS: handleSaveSettings(); break;
        case MSP_RESET_DEFAULTS: handleResetDefaults(); break;
        case MSP_REBOOT: handleReboot(); break;
        default: break; // unknown command - silently ignored
    }
}

} // namespace

void mspInit() {
    g_state = ParseState::IDLE;
    g_motorTestLastMs = 0;
}

void mspUpdate() {
    while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();

        switch (g_state) {
        case ParseState::IDLE:
            g_state = (b == '$') ? ParseState::GOT_DOLLAR : ParseState::IDLE;
            break;
        case ParseState::GOT_DOLLAR:
            g_state = (b == 'M') ? ParseState::GOT_M : ParseState::IDLE;
            break;
        case ParseState::GOT_M:
            // '<' = request from configurator to FC - the only direction we accept
            g_state = (b == '<') ? ParseState::GOT_DIR : ParseState::IDLE;
            break;
        case ParseState::GOT_DIR:
            g_expectedSize = b;
            g_checksum = b;
            g_state = (g_expectedSize <= FRAME_MAX_PAYLOAD) ? ParseState::GOT_SIZE : ParseState::IDLE;
            break;
        case ParseState::GOT_SIZE:
            g_cmd = b;
            g_checksum ^= b;
            g_payloadIdx = 0;
            g_state = (g_expectedSize == 0) ? ParseState::GOT_PAYLOAD : ParseState::GOT_CMD;
            break;
        case ParseState::GOT_CMD:
            g_payload[g_payloadIdx++] = b;
            g_checksum ^= b;
            if (g_payloadIdx >= g_expectedSize) g_state = ParseState::GOT_PAYLOAD;
            break;
        case ParseState::GOT_PAYLOAD:
            if (b == g_checksum) {
                dispatch(g_cmd, g_payload, g_expectedSize);
            }
            g_state = ParseState::IDLE;
            break;
        }
    }
}

bool mspGetMotorTestOverride(uint16_t outThrottle[4]) {
    const FlightState& fs = flightStateGet();
    if (fs.armed) return false;
    if (g_motorTestLastMs == 0 || (millis() - g_motorTestLastMs) > 1000) return false;
    for (int i = 0; i < 4; i++) outThrottle[i] = g_motorTestThrottle[i];
    return true;
}
