#pragma once
#include <stdint.h>

// ============================================================================
// Simple raw-flash blackbox logger, no filesystem. Writes fixed-size binary
// frames sequentially into the 'blackbox' partition (see partitions.csv).
//
// Simplification: logging restarts at partition offset 0 every boot (each
// flight overwrites the previous log) rather than maintaining a persisted
// ring-buffer head/tail across power cycles - avoids a whole class of "what
// if the pointer itself got corrupted" bugs for a feature that isn't safety
// critical. Download+save each log via the configurator before your next
// flight if you want to keep it.
// ============================================================================

#pragma pack(push, 1)
struct BlackboxFrame {
    uint32_t magic; // frame sync marker for the reader
    uint32_t timestampMs;
    float gyroDegS[3];
    float setpointDegS[3];
    float pidOutput[3];
    uint16_t motorThrottle[4];
    float batteryVoltage;
    uint8_t armed;
    uint8_t mode;
};
#pragma pack(pop)

void blackboxInit();

// Call from the flight loop when flightStateGet().blackboxEnabled is true.
// Internally rate-divided by settings.blackboxRateDivider.
void blackboxLogFrame(const float gyroDegS[3], const float setpointDegS[3],
                       const float pidOutput[3], const uint16_t motorThrottle[4],
                       float batteryVoltage, bool armed, uint8_t mode);

uint32_t blackboxGetWriteOffset();
uint32_t blackboxGetPartitionSize();

// Raw read for the MSP download command - the configurator reconstructs
// frames client-side by scanning for BlackboxFrame::magic.
bool blackboxReadRaw(uint32_t offset, uint8_t* outBuf, uint32_t len);

void blackboxEraseAll();
