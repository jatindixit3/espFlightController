# Code

All source code for the project. Two independent programs:

## `firmware/`

The flight controller firmware itself — a **PlatformIO** project targeting the
Seeed XIAO ESP32-S3 (Arduino framework). This is what runs on the board and
actually flies the quad: it reads the gyro, reads the receiver, runs the PID
loop, and drives the motors ~1000 times a second.

```bash
cd firmware
pio run              # build
pio run -t upload    # build + flash over USB-C
pio device monitor   # serial log (115200 baud)
```

Structure:

| Path | Purpose |
|------|---------|
| `platformio.ini` | build config (board, USB-CDC, partitions, warnings) |
| `partitions.csv` | flash layout, including the blackbox data partition |
| `include/*.h` | one header per subsystem |
| `src/*.cpp` | implementations |

Key modules: `imu` (MPU6050 + Mahony fusion + accel-gated calibration),
`sbus`, `dshot` (bidirectional DShot300), `esc_telemetry`, `mixer`, `pid`
(+ `filters`), `flight_state` (arming/failsafe/modes), `blackbox`, `ws2812` +
`indicators`, `msp` (the configurator protocol), and `settings` (NVS storage).

## `configurator/`

The browser-based tuning app — the Betaflight-Configurator-style UI. Pure static
HTML/CSS/JS, no build step. It talks to the firmware over USB with the Web Serial
API using the protocol in [`../../Documentation/PROTOCOL.md`](../../Documentation/PROTOCOL.md).

```bash
cd configurator
python -m http.server 8000
# open http://localhost:8000 in Chrome or Edge, click Connect
```

Layers: `js/msp-client.js` (Web Serial + wire-format encode/decode) →
`js/fc-api.js` (typed request/response wrapper) → `js/app.js` (the tabbed UI).

Both programs are MIT-licensed (see [`../LICENSE.md`](../LICENSE.md)).
