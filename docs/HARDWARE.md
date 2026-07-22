# Hardware: pinout and wiring rationale

Board: **Seeed Studio XIAO ESP32-S3** (standard, non-Sense variant - 11 GPIO
broken out as D0-D10, plus 5V/3V3/GND).
Gyro: **MPU6050** (I2C only).
ESCs: **AM32**, bidirectional DShot300 capable, with a separate telemetry
wire.
Receiver: **SBUS** input (from your own 2.4GHz/LoRa 433 link - the FC only
ever sees standard SBUS).
Airframe: 5" freestyle quad, 4S, ~2200mAh.

## Pinout

| XIAO Pin | GPIO | Assignment | Notes |
|---|---|---|---|
| D0 | GPIO1 | Motor 1 signal | DShot300 via RMT |
| D1 | GPIO2 | Motor 2 signal | DShot300 via RMT |
| D2 | GPIO3 | Motor 3 signal | DShot300 via RMT |
| D3 | GPIO4 | Motor 4 signal | DShot300 via RMT |
| D4 | GPIO5 | I2C SDA | -> MPU6050 |
| D5 | GPIO6 | I2C SCL | -> MPU6050 |
| D6 | GPIO43 | UART RX (telemetry) | <- ESC telemetry wire: voltage, current, temp, eRPM |
| D7 | GPIO44 | UART RX (inverted) | <- SBUS from receiver |
| D8 | GPIO7 | WS2812 data out | -> LED strip, via ~300 ohm series resistor |
| D9 | GPIO8 | Buzzer out | -> NPN transistor (2N3904/S8050) driving piezo buzzer |
| D10 | GPIO9 | Spare | Reserved |
| 5V | VBUS | Power in | From ESC BEC - see Power section below |
| 3V3 | 3V3_OUT | Power out | -> MPU6050 VCC (700mA available, MPU6050 draws ~4mA) |
| GND | - | Common ground | Tie ESC/battery GND, receiver GND, MPU6050 GND all together |
| USB-C | native | Configurator/CLI/flashing | Native USB-CDC |

Motor order follows Betaflight's default QuadX convention (M1 rear-right, M2
front-right, M3 rear-left, M4 front-left) - see `firmware/src/mixer.cpp`.
Physical wire-to-position mismatches are resolved via the configurator's
Motors tab (spin one at a time, props off), not by getting the firmware
table "correct" ahead of time.

## Why both bidirectional DShot AND the telemetry wire are wired up

Your AM32 ESCs support both. The firmware only *uses* the telemetry wire (see
`firmware/include/dshot.h` for the full reasoning) - bidirectional DShot's
return path needs a GCR-encoded bit-bang decode that's very hard to verify
without a logic analyzer and real ESCs on a bench. The telemetry wire gives
the same eRPM data (plus voltage/current/temp) over a simple, well-defined
UART protocol at a fraction of the risk. Wire both if your ESCs expose both
pads - it costs nothing and leaves bidirectional DShot available for a future
firmware revision if you want to add it yourself later.

## Power: the 5V pad has no reverse-protection diode

This is a documented fact about this specific board, not a design
preference: plugging in USB while the board is also powered from the ESC's
BEC (5V into the `5V` pad) can damage the board, because the two supplies
fight each other with no diode to arbitrate. The zero-extra-parts fix: never
have USB and the battery connected at the same time. Bench-tune over USB
with the battery disconnected; fly on battery only. If you want insurance
against forgetting, add a single small Schottky diode (anode to BEC output,
cathode to the `5V` pad) - optional, not required if you're disciplined
about not connecting both at once.

## Voltage/current sensing: from ESC telemetry, not a resistor divider

A raw LiPo (up to ~16.8V on 4S) cannot go directly into an ESP32 ADC pin
(3.3V max) without a divider - normally unavoidable for direct voltage
sensing. Since a divider counts as "an extra component" you asked to avoid,
and your ESC's telemetry packet already reports both pack voltage and
current (from its own onboard shunt), the firmware sources both from there
instead. Zero extra components, at the cost of the telemetry update rate
(tens of Hz) rather than instant ADC response - fine for battery-sag/
low-voltage warnings.

## The one real performance ceiling: gyro loop rate

MPU6050 is I2C-only, capped at 400kHz (Fast Mode). Real F7 boards use SPI
gyros (ICM42688P etc.) sampled at 8kHz-32kHz. This firmware runs the gyro/PID
loop around **1kHz**. For a 5" freestyle quad this is genuinely flyable and
can feel good with proper filtering - freestyle isn't as loop-rate-sensitive
as top-end racing - but it will not match an F7's raw loop rate or noise
floor. This is a sensor choice, not a firmware bug.

## Gyro+accel calibration

At boot (and on demand from the configurator's Setup tab), the firmware
samples the gyro for ~2s while the accelerometer confirms the board is
actually still (accel magnitude within tolerance of 1g, low gyro noise
throughout the window) before accepting a bias. If the board moves during
calibration, the window resets rather than baking in a bad bias - see
`firmware/src/imu.cpp` (`imuCalibrateGyro`).
