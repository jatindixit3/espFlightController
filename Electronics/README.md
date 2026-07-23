# Electronics

Pinout, wiring, and power for the flight controller. The full rationale behind
every choice (and the honest hardware caveats) is in
**[HARDWARE.md](HARDWARE.md)**.

## Pinout

Board: **Seeed Studio XIAO ESP32-S3** (non-Sense). Pin labels are the silkscreen
labels; GPIO is the ESP32-S3 pin the firmware actually uses (see
[`../Code/firmware/include/config.h`](../Code/firmware/include/config.h)).

| XIAO Pin | GPIO | Assignment | Notes |
|---|---|---|---|
| D0 | GPIO1 | Motor 1 signal | Bidirectional DShot300 (RMT) |
| D1 | GPIO2 | Motor 2 signal | Bidirectional DShot300 (RMT) |
| D2 | GPIO3 | Motor 3 signal | Bidirectional DShot300 (RMT) |
| D3 | GPIO4 | Motor 4 signal | Bidirectional DShot300 (RMT) |
| D4 | GPIO5 | I2C SDA | → MPU6050 |
| D5 | GPIO6 | I2C SCL | → MPU6050 |
| D6 | GPIO43 | UART RX (telemetry) | ← ESC telemetry wire (V/I/temp/eRPM) |
| D7 | GPIO44 | UART RX (inverted) | ← SBUS from receiver |
| D8 | GPIO7 | WS2812 data | → LED strip, via ~300 Ω series resistor (SPI-driven) |
| D9 | GPIO8 | Buzzer | → NPN transistor (e.g. 2N3904) → piezo buzzer |
| D10 | GPIO9 | Spare | reserved |
| 5V | — | Power in | from ESC BEC (see power note) |
| 3V3 | — | Power out | → MPU6050 VCC |
| GND | — | Common ground | tie ESC, receiver, and MPU6050 grounds together |
| USB-C | — | Configurator / flashing | native USB-CDC |

## Two things that will bite you if you skip them

1. **Never connect USB and the flight battery at the same time.** The XIAO's 5V
   pad has no reverse-protection diode, so a powered USB cable fighting the ESC
   BEC can damage the board. Bench-tune over USB with the battery disconnected;
   fly on battery only. (A single Schottky diode in the 5V line removes the risk
   if you want the insurance.)
2. **There is no dedicated voltage divider.** Battery voltage and current both
   come from the ESC telemetry wire, per the "no extra components" constraint —
   a raw 4S LiPo must never go straight into an ESP32 ADC pin.

Full reasoning, plus the calibration and loop-rate notes, in [HARDWARE.md](HARDWARE.md).
