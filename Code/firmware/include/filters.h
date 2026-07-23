#pragma once

// Standard RBJ audio-EQ-cookbook biquad filter (public domain formulas),
// used here for D-term/gyro lowpass filtering and RPM-tracking notch filters.
class Biquad {
public:
    void setLowpass(float freqHz, float sampleRateHz, float q = 0.707f);
    void setNotch(float freqHz, float sampleRateHz, float q = 2.0f);
    float process(float x);
    void reset();

private:
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
};
