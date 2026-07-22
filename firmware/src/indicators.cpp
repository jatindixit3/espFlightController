#include "indicators.h"
#include "config.h"
#include "settings.h"
#include "flight_state.h"
#include "esc_telemetry.h"
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

namespace {
Adafruit_NeoPixel g_strip(LED_COUNT, PIN_LED_DATA, NEO_GRB + NEO_KHZ800);

enum class BuzzerPattern { NONE, ARM, DISARM, BATTERY_WARNING, LOST_QUAD };
BuzzerPattern g_activePattern = BuzzerPattern::NONE;
uint32_t g_patternStartMs = 0;
bool g_wasArmed = false;

void setAllPixels(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < LED_COUNT; i++) g_strip.setPixelColor(i, g_strip.Color(r, g, b));
    g_strip.show();
}

void startPattern(BuzzerPattern p, uint32_t now) {
    if (g_activePattern != p) {
        g_activePattern = p;
        g_patternStartMs = now;
    }
}

// Non-blocking pattern player using tone()/noTone() - the flight loop must
// never be blocked by delay(), so all timing here is elapsed-time based.
void updateBuzzer(uint32_t now) {
    uint32_t elapsed = now - g_patternStartMs;

    switch (g_activePattern) {
    case BuzzerPattern::NONE:
        noTone(PIN_BUZZER);
        break;
    case BuzzerPattern::ARM:
        if (elapsed < 120) tone(PIN_BUZZER, 2000);
        else { noTone(PIN_BUZZER); g_activePattern = BuzzerPattern::NONE; }
        break;
    case BuzzerPattern::DISARM:
        if (elapsed < 80) tone(PIN_BUZZER, 1500);
        else if (elapsed < 160) noTone(PIN_BUZZER);
        else if (elapsed < 240) tone(PIN_BUZZER, 1500);
        else { noTone(PIN_BUZZER); g_activePattern = BuzzerPattern::NONE; }
        break;
    case BuzzerPattern::BATTERY_WARNING: {
        uint32_t cyclePos = elapsed % 1000;
        if (cyclePos < 150) tone(PIN_BUZZER, 2500); else noTone(PIN_BUZZER);
        break; // caller re-evaluates the condition and restarts/stops each update
    }
    case BuzzerPattern::LOST_QUAD: {
        uint32_t cyclePos = elapsed % 400;
        if (cyclePos < 200) tone(PIN_BUZZER, 3000); else noTone(PIN_BUZZER);
        break;
    }
    }
}
} // namespace

void indicatorsInit() {
    g_strip.begin();
    g_strip.setBrightness(80);
    setAllPixels(0, 0, 0);
    pinMode(PIN_BUZZER, OUTPUT);
    g_wasArmed = false;
    g_activePattern = BuzzerPattern::NONE;
}

void indicatorsUpdate() {
    uint32_t now = millis();
    const FlightState& fs = flightStateGet();
    Settings& s = settingsGet();
    float voltage = escTelemetryGetBatteryVoltage();

    int cellCount = s.batteryCellCountOverride;
    if (cellCount <= 0 && voltage > 0.0f) {
        cellCount = (int)((voltage / 3.7f) + 0.5f); // auto-detect: nearest nominal cell voltage
        if (cellCount < 1) cellCount = 1;
    }

    bool batteryCritical = false, batteryWarning = false;
    if (voltage > 0.0f && cellCount > 0) {
        float perCell = voltage / cellCount;
        batteryCritical = perCell < s.batteryCriticalVoltagePerCell;
        batteryWarning = !batteryCritical && perCell < s.batteryWarningVoltagePerCell;
    }

    // --- LED: failsafe > battery critical > armed (with warning overlay) > disarmed ---
    if (fs.failsafeActive) {
        bool on = (now / 150) % 2 == 0;
        setAllPixels(on ? 255 : 0, 0, 0);
    } else if (batteryCritical) {
        bool on = (now / 200) % 2 == 0;
        setAllPixels(on ? 255 : 0, on ? 40 : 0, 0);
    } else if (fs.armed) {
        if (batteryWarning) {
            bool on = (now / 500) % 2 == 0;
            setAllPixels(0, on ? 255 : 60, 0);
        } else {
            setAllPixels(0, 255, 0); // solid green while armed/flying
        }
    } else {
        setAllPixels(0, 0, 255); // solid blue while disarmed and healthy
    }

    // --- Buzzer: same priority order, plus arm/disarm edge beeps ---
    if (fs.failsafeActive) {
        startPattern(BuzzerPattern::LOST_QUAD, now);
    } else if (batteryCritical) {
        startPattern(BuzzerPattern::BATTERY_WARNING, now);
    } else if (fs.armed && !g_wasArmed) {
        startPattern(BuzzerPattern::ARM, now);
    } else if (!fs.armed && g_wasArmed) {
        startPattern(BuzzerPattern::DISARM, now);
    } else if (g_activePattern == BuzzerPattern::LOST_QUAD || g_activePattern == BuzzerPattern::BATTERY_WARNING) {
        g_activePattern = BuzzerPattern::NONE; // condition cleared
    }

    updateBuzzer(now);

    g_wasArmed = fs.armed;
}
