#include "filters.h"
#include <math.h>

void Biquad::setLowpass(float freqHz, float sampleRateHz, float q) {
    if (freqHz <= 0.0f || freqHz >= sampleRateHz * 0.5f) {
        // Degenerate request (filter disabled / invalid) - pass-through.
        b0 = 1; b1 = 0; b2 = 0; a1 = 0; a2 = 0;
        return;
    }
    float omega = 2.0f * (float)M_PI * freqHz / sampleRateHz;
    float alpha = sinf(omega) / (2.0f * q);
    float cosw = cosf(omega);

    float a0 = 1.0f + alpha;
    b0 = ((1.0f - cosw) / 2.0f) / a0;
    b1 = (1.0f - cosw) / a0;
    b2 = ((1.0f - cosw) / 2.0f) / a0;
    a1 = (-2.0f * cosw) / a0;
    a2 = (1.0f - alpha) / a0;
}

void Biquad::setNotch(float freqHz, float sampleRateHz, float q) {
    if (freqHz <= 0.0f || freqHz >= sampleRateHz * 0.5f) {
        b0 = 1; b1 = 0; b2 = 0; a1 = 0; a2 = 0;
        return;
    }
    float omega = 2.0f * (float)M_PI * freqHz / sampleRateHz;
    float alpha = sinf(omega) / (2.0f * q);
    float cosw = cosf(omega);

    float a0 = 1.0f + alpha;
    b0 = 1.0f / a0;
    b1 = (-2.0f * cosw) / a0;
    b2 = 1.0f / a0;
    a1 = (-2.0f * cosw) / a0;
    a2 = (1.0f - alpha) / a0;
}

float Biquad::process(float x) {
    float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = x;
    y2 = y1; y1 = y;
    return y;
}

void Biquad::reset() {
    x1 = x2 = y1 = y2 = 0;
}
