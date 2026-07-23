#pragma once
#include <stdint.h>

// ============================================================================
// ESC telemetry: KISS/BLHeli32/AM32-style 10-byte periodic packets on a single
// shared UART RX wire from a 4-in-1 ESC. No start byte in the protocol itself -
// framing is done by the inter-byte silence gap, same approach Betaflight's
// esc_sensor uses. Packets from the 4 motors arrive round-robin on the wire;
// we assign each successfully-CRC-checked packet to the next motor in
// sequence (1,2,3,4,1,2,3,4...).
//
// This is also our source of pack voltage and current (from the ESC's own
// shunt) - deliberately chosen over adding a dedicated ADC voltage divider,
// per your no-extra-components constraint. Trade-off: values update at the
// ESC's telemetry rate (tens of Hz), not instantly - fine for battery-sag/
// low-voltage warnings, not for anything needing sub-millisecond response.
// ============================================================================

struct MotorTelemetry {
    uint8_t temperatureC;
    float voltage;      // volts, whole-pack reading from this ESC's sense point
    float current;      // amps
    uint16_t consumptionMah;
    uint32_t eRpm;       // electrical RPM (raw value * 100)
    uint32_t lastUpdateMs;
};

void escTelemetryInit();

// Call frequently (every flight loop iteration or the dedicated telemetry
// task) - drains the UART FIFO and updates decoded packets as they complete.
void escTelemetryUpdate();

const MotorTelemetry& escTelemetryGet(int motorIndex);

// Best available battery voltage: average of all motors reporting fresh data
// in the last second, or 0 if none have reported yet.
float escTelemetryGetBatteryVoltage();

float escTelemetryGetTotalCurrent();
