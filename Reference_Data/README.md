# Reference Data

Datasheets, protocol specifications, and reference material this build relies on.
Where a spec has no single canonical PDF (DShot, SBUS), the most authoritative
community reference is linked.

## Board & sensor

- **Seeed Studio XIAO ESP32-S3** — pinout, power, and specs
  - Wiki / pinout: https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/
  - Pin multiplexing (the GPIO↔label map this project's pinout is built on): https://wiki.seeedstudio.com/xiao_esp32s3_pin_multiplexing/
- **Espressif ESP32-S3** — technical reference manual & datasheet: https://www.espressif.com/en/support/documents/technical-documents
  - RMT peripheral (used for DShot TX + bidirectional RX) is documented in the TRM.
- **InvenSense MPU-6050** — 6-axis gyro/accel, I2C
  - Datasheet (PS): https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
  - Register map (RM): https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

## Motor & ESC protocols

- **DShot** — digital ESC protocol (frame format, CRC, special commands)
  - Betaflight wiki: https://betaflight.com/docs/development/DshotOverview
  - Original spec discussion: https://github.com/betaflight/betaflight/blob/master/src/main/drivers/dshot.h
- **Bidirectional DShot / DShot telemetry (eRPM)** — the GCR-encoded return frame
  - https://github.com/bird-sanctuary/extended-dshot-telemetry
  - Betaflight bidirectional DShot notes: https://betaflight.com/docs/wiki/guides/current/DSHOT-RPM-Filtering
- **AM32 ESC firmware** — the ESC firmware these motors run: https://github.com/am32-firmware/AM32
- **KISS / BLHeli_32 ESC telemetry** — the 10-byte serial telemetry frame (voltage/current/temp/eRPM) parsed on the telemetry wire
  - Format reference: https://github.com/betaflight/betaflight/blob/master/src/main/sensors/esc_sensor.c

## Receiver

- **SBUS** — inverted UART, 100000 baud 8E2, 25-byte frame, 16×11-bit channels
  - Reverse-engineering reference: https://github.com/bolderflight/sbus

## Control & filtering

- **Betaflight** — the reference implementation whose *concepts* (PID+FF, rates
  curves, AUX mode ranges, RPM filtering, blackbox) this project reimplements:
  https://github.com/betaflight/betaflight
- **Mahony AHRS filter** — the gyro/accel fusion used for attitude:
  https://ahrs.readthedocs.io/en/latest/filters/mahony.html
- **RBJ Audio-EQ Cookbook** — the biquad filter formulas used for the gyro/D-term
  lowpass and RPM notch: https://www.w3.org/TR/audio-eq-cookbook/

## Indicators

- **WS2812 / WS2812B** addressable LED timing (driven here over SPI):
  https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf

> If you download any of these as PDFs for offline reference, drop them in this
> folder alongside this README.
