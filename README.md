# ESP32-S3 Flight Controller

A from-scratch Betaflight-style flight controller for the Seeed XIAO ESP32-S3,
using an MPU6050 gyro/accel, AM32 ESCs over bidirectional DShot300 +
ESC telemetry, and SBUS RC input - plus a browser-based tuning configurator
that talks to it, modeled on Betaflight Configurator (minus OSD, which was
explicitly out of scope).

**Read [docs/BENCH_TEST.md](docs/BENCH_TEST.md) before your first flight.**
This is new firmware on a hobby-grade gyro - bench-test it properly.

## What this is (and isn't)

This targets the same *feel* as an F4/F7 board running Betaflight - angle/
horizon/acro flight modes, PID + feedforward, RPM-aware filtering, blackbox,
LED/buzzer status, a full web tuning UI - built for a much cheaper part
(MPU6050 over I2C instead of an SPI gyro like the ICM42688P). One real,
unavoidable limit: I2C tops out around a 1kHz gyro/PID loop here, versus
8kHz+ on real F7 boards. For 5" freestyle flying that's still genuinely
flyable and can feel good with proper filtering; it will not match an F7's
raw loop rate or noise floor. See [docs/HARDWARE.md](docs/HARDWARE.md) for
the full pinout and the reasoning behind every hardware decision.

It is **not** a wire-compatible clone of real Betaflight firmware or the real
Betaflight Configurator - it's a custom, MSP-*framed* protocol (see
[docs/PROTOCOL.md](docs/PROTOCOL.md)) with its own command set, paired with
its own configurator built to match. It reuses Betaflight's *concepts*
(PID+FF, rates curves, AUX mode ranges, RPM filtering, blackbox) faithfully,
not its exact wire format or source code.

## Repo layout

```
firmware/       PlatformIO project (ESP32-S3, Arduino framework)
  platformio.ini
  partitions.csv      custom flash layout incl. the blackbox partition
  include/*.h          one header per subsystem
  src/*.cpp
configurator/   Static web app (no build step) - open configurator/index.html
  js/msp-client.js     Web Serial transport + wire-format encode/decode
  js/fc-api.js         typed request/response wrapper
  js/app.js            tabs UI
docs/
  HARDWARE.md          pinout, wiring rationale, power notes
  PROTOCOL.md          exact wire format (firmware <-> configurator)
  BENCH_TEST.md        pre-flight safety checklist - read this first
```

## Building and flashing the firmware

Requires [PlatformIO](https://platformio.org/) (`pip install platformio`, or
the VS Code extension).

```bash
cd firmware
pio run                # build
pio run -t upload      # build + flash over USB-C
pio device monitor      # serial log (115200 baud)
```

Compiles clean with `-Wall -Wextra` at the time of writing - if a PlatformIO/
Arduino-ESP32 core update introduces new warnings, treat them as blocking on
this specific project (it's a flight controller).

## Using the configurator

Web Serial (what the configurator uses to talk to the FC over USB) requires
a real HTTP origin or `localhost` - it will not work if you just double-click
`index.html` in some browsers. Serve it locally:

```bash
cd configurator
python -m http.server 8000
```

Then open `http://localhost:8000` in **Chrome or Edge** (Web Serial isn't
implemented in Firefox/Safari). Click **Connect**, pick the FC's serial port.

Tabs: Setup, Ports (informational - pins are fixed, see HARDWARE.md),
Configuration, PID Tuning, Rates, Receiver, Modes, Motors, Failsafe,
Blackbox, CLI (read-only settings dump, not an interactive shell - see the
tab itself for why). No OSD tab, per the original request.

## Key design decisions worth knowing about

- **Gyro calibration is gated on the accelerometer.** At boot (and on demand
  from the Setup tab), the firmware only accepts a gyro bias if the
  accelerometer confirms the board was genuinely still throughout the
  sampling window - otherwise it retries rather than baking in a bad bias
  from a board that was bumped mid-calibration.
- **Bidirectional DShot is implemented and on by default** (per explicit
  request), with the ESC telemetry wire kept as the voltage/current source
  and automatic RPM fallback. Every GCR response is CRC-checked - a decode
  failure can only degrade filter quality, never motor control - but the
  decode is unverified on real hardware until you run the bench check in
  `docs/BENCH_TEST.md` step 7. The toggle (Configuration tab) reverts to
  normal DShot300 with a save + reboot. A side effect: the WS2812 strip
  moved from the RMT-based Adafruit library to a custom SPI driver
  (`firmware/src/ws2812.cpp`), because motors now consume all 8 RMT
  channels - this also fixed a latent init-order conflict where the LED
  library could steal motor 1's RMT channel.
- **Battery voltage and current both come from ESC telemetry**, not a
  dedicated ADC voltage divider - per the "no extra components" constraint
  from the build spec. Trade-off: updates at the ESC's telemetry rate (tens
  of Hz), not instantly.
- **Failsafe is an immediate motor cut**, not a staged descent - there's no
  GPS/baro on this build to do anything smarter with, and "simplest safe
  default" beats a half-implemented fancy behavior.
- **Blackbox restarts from flash offset 0 every boot** (no persisted ring
  buffer across power cycles) - download and save a log before your next
  flight if you want to keep it.

## Known gaps / honest limitations

- No OSD (excluded by request).
- CLI tab is a read-only dump, not an interactive text command shell.
- Motor spin-direction reversal isn't wired up in firmware - set it via your
  AM32 configurator/passthrough, or swap two motor bell wires.
- Blackbox setpoint field currently logs raw stick input, not the post-
  rate-curve deg/s setpoint the PID loop actually used (documented in
  `docs/PROTOCOL.md`).
- This hasn't flown. It compiles clean and its logic has been reasoned
  through carefully, but "no compile errors" is not the same claim as
  "tuned and flight-proven" - that part is on you, on the bench, props off,
  before it ever sees a battery outdoors.
