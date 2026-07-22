#include "dshot.h"
#include "config.h"
#include <driver/rmt.h>

namespace {

// RMT clock: 80MHz APB / clk_div 2 = 40MHz -> 25ns per tick.
constexpr uint8_t RMT_CLK_DIV = 2;

// DShot300 bit timing (bit period 3.333us), in 25ns ticks.
constexpr uint16_t BIT_PERIOD_TICKS = 133;
constexpr uint16_t T1H_TICKS = 89;
constexpr uint16_t T1L_TICKS = BIT_PERIOD_TICKS - T1H_TICKS;
constexpr uint16_t T0H_TICKS = 44;
constexpr uint16_t T0L_TICKS = BIT_PERIOD_TICKS - T0H_TICKS;

rmt_channel_t g_channels[4];

uint16_t buildDshotFrame(uint16_t throttleValue11bit, bool telemetryRequest) {
    uint16_t packet = (uint16_t)((throttleValue11bit << 1) | (telemetryRequest ? 1 : 0));
    uint16_t csumData = packet;
    uint16_t csum = 0;
    for (int i = 0; i < 3; i++) {
        csum ^= csumData;
        csumData >>= 4;
    }
    csum &= 0x0F;
    return (uint16_t)((packet << 4) | csum);
}

void frameToItems(uint16_t frame, rmt_item32_t items[16]) {
    for (int i = 0; i < 16; i++) {
        bool bit = (frame >> (15 - i)) & 0x1;
        items[i].level0 = 1;
        items[i].duration0 = bit ? T1H_TICKS : T0H_TICKS;
        items[i].level1 = 0;
        items[i].duration1 = bit ? T1L_TICKS : T0L_TICKS;
    }
}

void writeMotor(int index, uint16_t throttleValue) {
    // throttleValue == 0 means "stop" and is sent as DShot command 0, which is
    // within the special-command range (0-47) and universally means motor stop.
    uint16_t frame = buildDshotFrame(throttleValue, false);
    rmt_item32_t items[16];
    frameToItems(frame, items);
    rmt_write_items(g_channels[index], items, 16, false);
}

} // namespace

void dshotInit() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_channels[i] = (rmt_channel_t)i;

        rmt_config_t cfg = {};
        cfg.rmt_mode = RMT_MODE_TX;
        cfg.channel = g_channels[i];
        cfg.gpio_num = (gpio_num_t)MOTOR_PINS[i];
        cfg.clk_div = RMT_CLK_DIV;
        cfg.mem_block_num = 1;
        cfg.flags = 0;
        cfg.tx_config.carrier_en = false;
        cfg.tx_config.loop_en = false;
        cfg.tx_config.idle_output_en = true;
        cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW; // DShot line idles low
        cfg.tx_config.carrier_freq_hz = 38000;
        cfg.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
        cfg.tx_config.carrier_duty_percent = 33;

        rmt_config(&cfg);
        rmt_driver_install(g_channels[i], 0, 0);
    }
    dshotStopAll();
}

void dshotWriteThrottles(const uint16_t throttle[4]) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        writeMotor(i, throttle[i]);
    }
}

void dshotStopAll() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        writeMotor(i, 0);
    }
}
