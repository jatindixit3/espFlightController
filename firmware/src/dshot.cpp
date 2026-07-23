#include "dshot.h"
#include "config.h"
#include "settings.h"
#include <Arduino.h>
#include <driver/rmt.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>

namespace {

// RMT clock: 80MHz APB / clk_div 2 = 40MHz -> 25ns per tick.
constexpr uint8_t RMT_CLK_DIV = 2;

// DShot300 bit timing (bit period 3.333us), in 25ns ticks.
constexpr uint16_t BIT_PERIOD_TICKS = 133;
constexpr uint16_t T1H_TICKS = 89;
constexpr uint16_t T0H_TICKS = 44;

// Bidir response: 21 bits at 5/4 the TX bitrate -> 375kbps -> ~2.667us/bit.
constexpr float GCR_BIT_TICKS = 106.7f;      // 2.667us in 25ns ticks
constexpr uint16_t RX_IDLE_THRESHOLD_TICKS = 1200; // 30us of quiet line ends a capture
constexpr uint32_t BIDIR_RESPONSE_BITS = 21;

// Inverse GCR map: 5-bit quintet -> 4-bit nibble, 0xFF = invalid encoding.
constexpr uint8_t GCR_NIBBLE[32] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F,
    0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07,
    0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF,
};

bool g_bidir = false;
rmt_channel_t g_txChannels[4];
rmt_channel_t g_rxChannels[4]; // S3 RMT channels 4-7 are the RX-capable ones
RingbufHandle_t g_rxRingbuf[4] = {nullptr, nullptr, nullptr, nullptr};
DshotTelemetry g_telemetry[4];

uint16_t buildDshotFrame(uint16_t throttleValue11bit, bool telemetryRequest) {
    uint16_t packet = (uint16_t)((throttleValue11bit << 1) | (telemetryRequest ? 1 : 0));
    uint16_t csumData = packet;
    uint16_t csum = 0;
    for (int i = 0; i < 3; i++) {
        csum ^= csumData;
        csumData >>= 4;
    }
    if (g_bidir) csum = (uint16_t)~csum; // complemented checksum = "answer with eRPM"
    return (uint16_t)((packet << 4) | (csum & 0x0F));
}

void frameToItems(uint16_t frame, rmt_item32_t items[16]) {
    // Normal DShot: bit starts high, line idles low. Bidirectional DShot is
    // the same waveform inverted: bit starts low, line idles high.
    const uint8_t activeLevel = g_bidir ? 0 : 1;
    for (int i = 0; i < 16; i++) {
        bool bit = (frame >> (15 - i)) & 0x1;
        uint16_t activeTicks = bit ? T1H_TICKS : T0H_TICKS;
        items[i].level0 = activeLevel;
        items[i].duration0 = activeTicks;
        items[i].level1 = activeLevel ^ 1;
        items[i].duration1 = BIT_PERIOD_TICKS - activeTicks;
    }
}

void writeMotor(int index, uint16_t throttleValue) {
    // throttleValue == 0 means "stop" and is sent as DShot command 0, which is
    // within the special-command range (0-47) and universally means motor stop.
    uint16_t frame = buildDshotFrame(throttleValue, false);
    rmt_item32_t items[16];
    frameToItems(frame, items);
    rmt_write_items(g_txChannels[index], items, 16, false);
}

// Decodes one captured RMT item sequence into an eRPM value. Returns false on
// anything malformed - truncated capture, invalid GCR quintet, bad CRC - so a
// noise burst can never inject a fake RPM into the filter.
bool decodeGcrResponse(const rmt_item32_t* items, size_t count, uint32_t* eRpmOut) {
    uint32_t bits = 0;
    uint32_t bitCount = 0;

    for (size_t i = 0; i < count && bitCount < BIDIR_RESPONSE_BITS; i++) {
        const uint16_t durations[2] = {(uint16_t)items[i].duration0, (uint16_t)items[i].duration1};
        const uint8_t levels[2] = {(uint8_t)items[i].level0, (uint8_t)items[i].level1};
        for (int half = 0; half < 2; half++) {
            if (durations[half] == 0) { i = count; break; } // end-of-capture marker
            int bitsInRun = (int)((durations[half] / GCR_BIT_TICKS) + 0.5f);
            if (bitsInRun <= 0) bitsInRun = 1;
            for (int b = 0; b < bitsInRun && bitCount < BIDIR_RESPONSE_BITS; b++) {
                bits = (bits << 1) | levels[half];
                bitCount++;
            }
        }
    }

    if (bitCount < 4) return false; // clearly not a response (e.g. pure idle capture)

    // The final high period before the line returns to idle is often absorbed
    // into the end-of-capture marker - pad the tail with idle-high bits.
    while (bitCount < BIDIR_RESPONSE_BITS) {
        bits = (bits << 1) | 1;
        bitCount++;
    }

    if (bits & (1u << 20)) return false; // start bit must be 0

    uint32_t gcr = (bits ^ (bits >> 1)) & 0xFFFFF; // edge decode -> 20-bit GCR stream

    uint16_t value = 0;
    for (int q = 0; q < 4; q++) {
        uint8_t nibble = GCR_NIBBLE[(gcr >> (15 - 5 * q)) & 0x1F];
        if (nibble == 0xFF) return false;
        value = (uint16_t)((value << 4) | nibble);
    }

    // Transmitted CRC nibble is complemented, so all four nibbles XOR to 0xF.
    if (((value ^ (value >> 4) ^ (value >> 8) ^ (value >> 12)) & 0x0F) != 0x0F) return false;

    uint16_t payload = value >> 4; // 12 bits: eee mmmmmmmmm (period, us)
    if (payload == 0x0FFF) { // "not spinning" marker
        *eRpmOut = 0;
        return true;
    }
    uint32_t periodUs = (uint32_t)(payload & 0x1FF) << (payload >> 9);
    if (periodUs == 0) return false;
    *eRpmOut = 60000000u / periodUs;
    return true;
}

void drainResponses() {
    uint32_t now = millis();
    for (int m = 0; m < 4; m++) {
        if (!g_rxRingbuf[m]) continue;
        size_t len = 0;
        void* data;
        // Multiple captures can be pending (e.g. a pre-response idle stretch
        // plus the real frame) - decode them all, keep whatever passes CRC.
        while ((data = xRingbufferReceive(g_rxRingbuf[m], &len, 0)) != nullptr) {
            uint32_t eRpm;
            if (len >= sizeof(rmt_item32_t) &&
                decodeGcrResponse((const rmt_item32_t*)data, len / sizeof(rmt_item32_t), &eRpm)) {
                g_telemetry[m].eRpm = eRpm;
                g_telemetry[m].lastUpdateMs = now;
            }
            vRingbufferReturnItem(g_rxRingbuf[m], data);
            len = 0;
        }
    }
}

} // namespace

void dshotInit() {
    g_bidir = settingsGet().bidirDshotEnabled;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_txChannels[i] = (rmt_channel_t)i;
        g_telemetry[i] = {};

        if (g_bidir) {
            // Configure the RX side FIRST: its rmt_config() routes the pin
            // into the RX unit via the GPIO matrix. The TX config below then
            // routes the output; matrix input routing survives it, and the
            // final gpio_set_direction(INPUT_OUTPUT) enables both buffers.
            g_rxChannels[i] = (rmt_channel_t)(4 + i);
            rmt_config_t rx = {};
            rx.rmt_mode = RMT_MODE_RX;
            rx.channel = g_rxChannels[i];
            rx.gpio_num = (gpio_num_t)MOTOR_PINS[i];
            rx.clk_div = RMT_CLK_DIV;
            rx.mem_block_num = 1;
            rx.flags = 0;
            rx.rx_config.idle_threshold = RX_IDLE_THRESHOLD_TICKS;
            rx.rx_config.filter_ticks_thresh = 100; // ~1.25us glitch filter (APB ticks), min real pulse is 2.67us
            rx.rx_config.filter_en = true;
            rmt_config(&rx);
            rmt_driver_install(g_rxChannels[i], 512, 0);
            rmt_get_ringbuf_handle(g_rxChannels[i], &g_rxRingbuf[i]);
        }

        rmt_config_t cfg = {};
        cfg.rmt_mode = RMT_MODE_TX;
        cfg.channel = g_txChannels[i];
        cfg.gpio_num = (gpio_num_t)MOTOR_PINS[i];
        cfg.clk_div = RMT_CLK_DIV;
        cfg.mem_block_num = 1;
        cfg.flags = 0;
        cfg.tx_config.carrier_en = false;
        cfg.tx_config.loop_en = false;
        cfg.tx_config.idle_output_en = true;
        cfg.tx_config.idle_level = g_bidir ? RMT_IDLE_LEVEL_HIGH : RMT_IDLE_LEVEL_LOW;
        cfg.tx_config.carrier_freq_hz = 38000;
        cfg.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
        cfg.tx_config.carrier_duty_percent = 33;
        rmt_config(&cfg);
        rmt_driver_install(g_txChannels[i], 0, 0);

        if (g_bidir) {
            // Pull-up keeps the line at its idle-high level while we tristate
            // it for the ESC's response window.
            gpio_set_pull_mode((gpio_num_t)MOTOR_PINS[i], GPIO_PULLUP_ONLY);
            gpio_set_direction((gpio_num_t)MOTOR_PINS[i], GPIO_MODE_INPUT_OUTPUT);
        }
    }
    dshotStopAll();
}

void dshotWriteThrottles(const uint16_t throttle[4]) {
    if (g_bidir) {
        // Harvest whatever responses arrived since the previous frame, then
        // quiesce RX so it doesn't capture our own outgoing TX.
        drainResponses();
        for (int i = 0; i < MOTOR_COUNT; i++) rmt_rx_stop(g_rxChannels[i]);
        for (int i = 0; i < MOTOR_COUNT; i++) {
            gpio_set_direction((gpio_num_t)MOTOR_PINS[i], GPIO_MODE_INPUT_OUTPUT);
        }
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        writeMotor(i, throttle[i]);
    }

    if (g_bidir) {
        // All four frames run in parallel (~53us); wait for completion, then
        // release the lines and arm capture before the ESC's ~30us turnaround
        // ends. A missed window just drops that cycle's sample - the decode
        // CRC rejects partial captures.
        for (int i = 0; i < MOTOR_COUNT; i++) rmt_wait_tx_done(g_txChannels[i], pdMS_TO_TICKS(1));
        for (int i = 0; i < MOTOR_COUNT; i++) {
            gpio_set_direction((gpio_num_t)MOTOR_PINS[i], GPIO_MODE_INPUT);
        }
        for (int i = 0; i < MOTOR_COUNT; i++) rmt_rx_start(g_rxChannels[i], true);
    }
}

void dshotStopAll() {
    const uint16_t zeros[4] = {0, 0, 0, 0};
    dshotWriteThrottles(zeros);
}

const DshotTelemetry& dshotGetTelemetry(int motorIndex) {
    return g_telemetry[motorIndex];
}
