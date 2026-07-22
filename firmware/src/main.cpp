#include <Arduino.h>
#include "config.h"
#include "settings.h"
#include "imu.h"
#include "sbus.h"
#include "dshot.h"
#include "esc_telemetry.h"
#include "mixer.h"
#include "pid.h"
#include "flight_state.h"
#include "blackbox.h"
#include "indicators.h"
#include "msp.h"

namespace {

void faultLoop(const char* reason) {
    // IMU bring-up failed (wiring fault) - never start flight tasks, so
    // motors can never receive a command. Blink red and report over USB so
    // the problem is visible even without a working attitude estimate.
    Serial.begin(115200);
    while (true) {
        Serial.println(reason);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(150);
        digitalWrite(LED_BUILTIN, LOW);
        delay(150);
    }
}

void flightTask(void* /*param*/) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1); // ~1kHz; FLIGHT_LOOP_PERIOD_US used for dt math below
    uint32_t lastMicros = micros();

    for (;;) {
        vTaskDelayUntil(&lastWake, period);

        uint32_t nowMicros = micros();
        float dt = (nowMicros - lastMicros) / 1000000.0f;
        if (dt <= 0.0f || dt > 0.01f) dt = (float)FLIGHT_LOOP_PERIOD_US / 1000000.0f;
        lastMicros = nowMicros;

        sbusUpdate();
        flightStateUpdate();
        imuUpdate(dt);

        const FlightState& fs = flightStateGet();

        float rawSetpoint[3]; // for blackbox only; pidUpdate recomputes its own internally
        rawSetpoint[0] = fs.rcCommand[0];
        rawSetpoint[1] = fs.rcCommand[1];
        rawSetpoint[2] = fs.rcCommand[2];

        PidOutput pidOut = pidUpdate(fs.rcCommand, fs.mode, dt);

        uint16_t motorThrottle[4];
        uint16_t testOverride[4];
        if (mspGetMotorTestOverride(testOverride)) {
            for (int i = 0; i < 4; i++) motorThrottle[i] = testOverride[i];
        } else {
            MixerInput mixIn{fs.throttle, pidOut.roll, pidOut.pitch, pidOut.yaw};
            mixerCompute(mixIn, fs.armed, motorThrottle);
        }

        dshotWriteThrottles(motorThrottle);

        if (fs.blackboxEnabled) {
            const ImuSample& imu = imuGetSample();
            float pidArr[3] = {pidOut.roll, pidOut.pitch, pidOut.yaw};
            blackboxLogFrame(imu.gyroDegS, rawSetpoint, pidArr, motorThrottle,
                              escTelemetryGetBatteryVoltage(), fs.armed, (uint8_t)fs.mode);
        }
    }
}

void commsTask(void* /*param*/) {
    for (;;) {
        escTelemetryUpdate();
        mspUpdate();
        indicatorsUpdate();
        vTaskDelay(pdMS_TO_TICKS(10)); // ~100Hz - plenty for telemetry/UI/indicators
    }
}

} // namespace

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    settingsInit();
    Settings& s = settingsGet();

    indicatorsInit();

    if (!imuInit()) {
        faultLoop("FATAL: MPU6050 not responding - check wiring (SDA=D4/GPIO5, SCL=D5/GPIO6)");
    }
    imuSetBoardAlignment(s.boardAlignRollDeg, s.boardAlignPitchDeg, s.boardAlignYawDeg);

    sbusInit();
    dshotInit();
    escTelemetryInit();
    mixerInit();
    pidInit();
    flightStateInit();
    blackboxInit();
    mspInit();

    // Auto-calibrate at boot, same expectation as any Betaflight board: hold
    // still while powering on. Gated on the accelerometer confirming
    // stillness (see imu.cpp) - if the board is moved, this simply retries
    // until it times out, and imuIsCalibrated() stays false (blocking arming)
    // until a successful MSP_CALIBRATE_GYRO retry from the configurator.
    imuCalibrateGyro();

    xTaskCreatePinnedToCore(flightTask, "flight", 8192, nullptr, 24, nullptr, 1);
    xTaskCreatePinnedToCore(commsTask, "comms", 8192, nullptr, 10, nullptr, 0);
}

void loop() {
    // Everything runs in the two FreeRTOS tasks created in setup(); nothing
    // to do on the Arduino loop task itself.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
