#include "blackbox.h"
#include "config.h"
#include "settings.h"
#include <esp_partition.h>
#include <Arduino.h>
#include <string.h>

namespace {
constexpr uint32_t FRAME_MAGIC = 0x424C4B58; // "BLKX"
constexpr uint32_t SECTOR_SIZE = 4096;

const esp_partition_t* g_partition = nullptr;
uint32_t g_writeOffset = 0;
uint32_t g_erasedUpToSector = 0; // next sector index that still needs erasing this lap
uint32_t g_loopCounter = 0;

void ensureErased(uint32_t offset, uint32_t length) {
    if (!g_partition || length == 0) return;
    uint32_t lastByte = offset + length - 1;
    uint32_t neededSector = lastByte / SECTOR_SIZE;
    while (g_erasedUpToSector <= neededSector) {
        esp_partition_erase_range(g_partition, g_erasedUpToSector * SECTOR_SIZE, SECTOR_SIZE);
        g_erasedUpToSector++;
    }
}
} // namespace

void blackboxInit() {
    g_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                            (esp_partition_subtype_t)0x40,
                                            BLACKBOX_PARTITION_NAME);
    g_writeOffset = 0;
    g_erasedUpToSector = 0;
    g_loopCounter = 0;
    if (g_partition) {
        ensureErased(0, sizeof(BlackboxFrame));
    }
}

void blackboxLogFrame(const float gyroDegS[3], const float setpointDegS[3],
                       const float pidOutput[3], const uint16_t motorThrottle[4],
                       float batteryVoltage, bool armed, uint8_t mode) {
    if (!g_partition) return;

    Settings& s = settingsGet();
    g_loopCounter++;
    if (s.blackboxRateDivider > 1 && (g_loopCounter % s.blackboxRateDivider) != 0) return;

    BlackboxFrame frame;
    frame.magic = FRAME_MAGIC;
    frame.timestampMs = millis();
    memcpy(frame.gyroDegS, gyroDegS, sizeof(frame.gyroDegS));
    memcpy(frame.setpointDegS, setpointDegS, sizeof(frame.setpointDegS));
    memcpy(frame.pidOutput, pidOutput, sizeof(frame.pidOutput));
    memcpy(frame.motorThrottle, motorThrottle, sizeof(frame.motorThrottle));
    frame.batteryVoltage = batteryVoltage;
    frame.armed = armed ? 1 : 0;
    frame.mode = mode;

    uint32_t frameSize = sizeof(BlackboxFrame);
    if (g_writeOffset + frameSize > g_partition->size) {
        g_writeOffset = 0;
        g_erasedUpToSector = 0; // start re-erasing from the beginning on wrap
    }

    ensureErased(g_writeOffset, frameSize);
    esp_partition_write(g_partition, g_writeOffset, &frame, frameSize);
    g_writeOffset += frameSize;
}

uint32_t blackboxGetWriteOffset() {
    return g_writeOffset;
}

uint32_t blackboxGetPartitionSize() {
    return g_partition ? g_partition->size : 0;
}

bool blackboxReadRaw(uint32_t offset, uint8_t* outBuf, uint32_t len) {
    if (!g_partition || offset + len > g_partition->size) return false;
    return esp_partition_read(g_partition, offset, outBuf, len) == ESP_OK;
}

void blackboxEraseAll() {
    if (!g_partition) return;
    esp_partition_erase_range(g_partition, 0, g_partition->size);
    g_writeOffset = 0;
    g_erasedUpToSector = g_partition->size / SECTOR_SIZE;
}
