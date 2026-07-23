# IS2026 Spring - ESP32-S3 Flight Controller

A from-scratch, Betaflight-style flight controller for a 5" freestyle quadcopter,
built on the **Seeed Studio XIAO ESP32-S3**. It runs a real 1&nbsp;kHz flight loop
(gyro + accelerometer fusion, PID + feedforward, RPM-aware filtering, blackbox
logging, LED/buzzer status) and ships with its own **browser-based tuning
configurator** modeled on Betaflight Configurator - talking to the board over USB
using a custom MSP-style protocol. The one deliberate omission is OSD.

- **Gyro:** MPU6050 (I2C)
- **ESCs:** AM32, bidirectional DShot300 + telemetry wire
- **Receiver:** SBUS
- **Airframe:** 5" freestyle quad, 4S, ~2200&nbsp;mAh
- **Status LEDs / buzzer:** WS2812 (over SPI) + piezo buzzer

> ⚠️ This firmware has **not flown yet**. It compiles clean and its logic has been
> reasoned through carefully, but "no compile errors" is not "flight-proven."
> Work through **[Documentation/BENCH_TEST.md](Documentation/BENCH_TEST.md)** on the
> bench, props off, before it ever sees a battery outdoors.

## Repository contents

| Folder | Contents |
|--------|----------|
| **[Code/](Code/)** | All source code: the `firmware/` PlatformIO project (the flight controller itself) and the `configurator/` web app (the tuning UI). |
| **[Electronics/](Electronics/)** | Full pinout, wiring rationale, power notes, and the pinout diagram. |
| **[Documentation/](Documentation/)** | Project overview and build guide, the FC&harr;configurator wire protocol, the pre-flight bench-test checklist, and the system flow diagram. |
| **[Reference_Data/](Reference_Data/)** | Datasheets and specifications the build relies on (XIAO ESP32-S3, MPU6050, SBUS, DShot / bidirectional DShot, AM32, etc.). |
| **[Photos_Videos/](Photos_Videos/)** | Build photos, prototype progress, and demo videos. |
| **[BOM.csv](BOM.csv)** | Bill of materials. |

## Quick start

**Build & flash the firmware** (requires [PlatformIO](https://platformio.org/)):

```bash
cd Code/firmware
pio run              # build
pio run -t upload    # build + flash over USB-C
```

**Open the configurator** (Chrome or Edge - it uses the Web Serial API, which
needs a real origin, not a double-clicked file):

```bash
cd Code/configurator
python -m http.server 8000
# then open http://localhost:8000 and click Connect
```

See **[Documentation/README.md](Documentation/README.md)** for the full walkthrough.

## License

Licenses

<a href="LICENSE.md"><img src="Licenses_facts.svg" width="400" alt="Open Source Licenses Facts"/></a>

#### Hardware
CERN Open Hardware License Version 2 - Strongly Reciprocal ([CERN-OHL-S-2.0](https://spdx.org/licenses/CERN-OHL-S-2.0.html)).

#### Software
MIT open source [license](http://opensource.org/licenses/MIT).

#### Documentation:
<a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by/4.0/">Creative Commons Attribution 4.0 International License</a>.

---

## 📬 Contact/Team

> _List team members and contact emails or GitHub profiles._

- **Jatin Dixit** — jatin.dixit2013@gmail.com — _[add your GitHub handle here]_
>
> ---
