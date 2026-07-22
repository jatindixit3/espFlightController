#include "pid.h"
#include "imu.h"
#include "settings.h"
#include "esc_telemetry.h"
#include "filters.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

namespace {

// PID gains use Betaflight-familiar numbers (defaults like P=45, I=80) so the
// feel of "turn this up a bit" transfers, but they are NOT numerically
// identical to real Betaflight internals - impossible to be, since the whole
// control computation (filters, scaling, fixed point vs float) differs. This
// constant maps the raw gain*error math into the mixer's -1..1-ish output
// range. Retune from the shipped defaults via the configurator.
constexpr float PID_OUTPUT_SCALE = 1.0f / 15000.0f;
constexpr float I_LIMIT = 8000.0f; // pre-scale integrator clamp (~0.53 post-scale)

struct AxisState {
    float integrator = 0;
    float prevMeasurement = 0; // for D-term (computed on measurement, not error)
    float prevSetpoint = 0;    // for feedforward
    Biquad gyroLowpass;
    Biquad dtermLowpass;
    Biquad rpmNotch;
};

AxisState g_axis[3]; // 0=roll, 1=pitch, 2=yaw
float g_lastNotchFreq = 0.0f;

float applyRates(float stickInput, const RateProfile& rp) {
    float expoApplied = stickInput * (1.0f - rp.expo) + stickInput * stickInput * stickInput * rp.expo;
    float angleRate = 200.0f * rp.rcRate * expoApplied;
    if (rp.superRate > 0.0f) {
        float absExpo = fabsf(expoApplied);
        float factor = 1.0f / fmaxf(1.0f - absExpo * rp.superRate, 0.01f);
        angleRate *= factor;
    }
    return angleRate; // deg/s
}

float computeMotorAvgHz(const Settings& s) {
    uint32_t sum = 0;
    int count = 0;
    uint32_t now = millis();
    for (int m = 0; m < 4; m++) {
        const MotorTelemetry& t = escTelemetryGet(m);
        if (t.lastUpdateMs != 0 && (now - t.lastUpdateMs) < 500) {
            sum += t.eRpm;
            count++;
        }
    }
    if (count == 0 || s.motorPolePairs == 0) return 0.0f;
    float avgErpm = (float)sum / (float)count;
    float mechanicalRpm = avgErpm / (float)s.motorPolePairs;
    return mechanicalRpm / 60.0f; // Hz
}

} // namespace

void pidInit() {
    pidResetState();
}

void pidResetState() {
    for (int i = 0; i < 3; i++) {
        g_axis[i].integrator = 0;
        g_axis[i].prevMeasurement = 0;
        g_axis[i].prevSetpoint = 0;
        g_axis[i].gyroLowpass.reset();
        g_axis[i].dtermLowpass.reset();
        g_axis[i].rpmNotch.reset();
    }
}

PidOutput pidUpdate(const float rcCommand[3], FlightMode mode, float dtSeconds) {
    Settings& s = settingsGet();
    const ImuSample& imu = imuGetSample();

    // Filter coefficients rarely change (only via the configurator) - cheap
    // enough to just recompute whenever the target parameter differs from
    // last call rather than tracking explicit dirty flags.
    static float lastGyroLpHz = -1, lastDtermLpHz = -1;
    if (s.gyroLowpassHz != lastGyroLpHz) {
        for (int i = 0; i < 3; i++) g_axis[i].gyroLowpass.setLowpass(s.gyroLowpassHz, (float)FLIGHT_LOOP_HZ);
        lastGyroLpHz = s.gyroLowpassHz;
    }
    if (s.dtermLowpassHz != lastDtermLpHz) {
        for (int i = 0; i < 3; i++) g_axis[i].dtermLowpass.setLowpass(s.dtermLowpassHz, (float)FLIGHT_LOOP_HZ);
        lastDtermLpHz = s.dtermLowpassHz;
    }

    if (s.rpmFilterEnabled) {
        float motorHz = computeMotorAvgHz(s);
        if (motorHz > 10.0f && fabsf(motorHz - g_lastNotchFreq) > 2.0f) {
            for (int i = 0; i < 3; i++) g_axis[i].rpmNotch.setNotch(motorHz, (float)FLIGHT_LOOP_HZ, 3.0f);
            g_lastNotchFreq = motorHz;
        } else if (motorHz <= 10.0f && g_lastNotchFreq != 0.0f) {
            // No healthy RPM data - bypass rather than filter at a stale frequency.
            for (int i = 0; i < 3; i++) g_axis[i].rpmNotch.setNotch(0, (float)FLIGHT_LOOP_HZ);
            g_lastNotchFreq = 0.0f;
        }
    }

    float rawSetpoint[3];
    rawSetpoint[0] = applyRates(rcCommand[0], s.rates[0]);
    rawSetpoint[1] = applyRates(rcCommand[1], s.rates[1]);
    rawSetpoint[2] = applyRates(rcCommand[2], s.rates[2]);

    float setpoint[3] = {rawSetpoint[0], rawSetpoint[1], rawSetpoint[2]};

    if (mode == FLIGHT_MODE_ANGLE || mode == FLIGHT_MODE_HORIZON) {
        float desiredAngle[2] = {
            rcCommand[0] * s.maxAngleDeg,
            rcCommand[1] * s.maxAngleDeg
        };
        for (int axis = 0; axis < 2; axis++) {
            float angleError = desiredAngle[axis] - imu.attitudeDeg[axis];
            float levelRate = angleError * s.levelGainP;

            if (mode == FLIGHT_MODE_ANGLE) {
                setpoint[axis] = levelRate;
            } else {
                // Horizon: near-center stick leans on leveling, full deflection is pure acro.
                float stickAbs = fabsf(rcCommand[axis]);
                float t = s.horizonTiltEffect / 100.0f;
                float acroWeight = stickAbs * t + stickAbs * stickAbs * (1.0f - t);
                if (acroWeight > 1.0f) acroWeight = 1.0f;
                setpoint[axis] = levelRate * (1.0f - acroWeight) + rawSetpoint[axis] * acroWeight;
            }
        }
    }

    PidOutput out{0, 0, 0};
    float* outAxis[3] = {&out.roll, &out.pitch, &out.yaw};

    for (int axis = 0; axis < 3; axis++) {
        AxisState& st = g_axis[axis];
        const AxisPid& gains = s.pid[axis];

        float measurement = imu.gyroDegS[axis];
        if (s.rpmFilterEnabled) measurement = st.rpmNotch.process(measurement);
        measurement = st.gyroLowpass.process(measurement);

        float error = setpoint[axis] - measurement;

        float pTerm = gains.P * error;

        st.integrator += gains.I * error * dtSeconds;
        if (st.integrator > I_LIMIT) st.integrator = I_LIMIT;
        if (st.integrator < -I_LIMIT) st.integrator = -I_LIMIT;
        float iTerm = st.integrator;

        float measurementDelta = (measurement - st.prevMeasurement) / dtSeconds;
        float dTermRaw = -gains.D * measurementDelta; // D on measurement, avoids setpoint-change kick
        float dTerm = st.dtermLowpass.process(dTermRaw);
        st.prevMeasurement = measurement;

        float setpointDelta = (setpoint[axis] - st.prevSetpoint) / dtSeconds;
        float ffTerm = gains.FF * setpointDelta;
        st.prevSetpoint = setpoint[axis];

        *outAxis[axis] = (pTerm + iTerm + dTerm + ffTerm) * PID_OUTPUT_SCALE;
    }

    return out;
}
