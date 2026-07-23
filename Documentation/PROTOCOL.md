# FC-Configurator wire protocol

Custom binary protocol, framed like classic MSPv1 but with its own command
set. Implemented in `Code/firmware/src/msp.cpp` (server) and
`Code/configurator/js/msp-client.js` (client). Transport: native USB-CDC serial
(Web Serial API in the browser), 115200 baud is irrelevant for USB-CDC but
the port is opened at that rate for compatibility.

All multi-byte fields are **little-endian** (native ESP32 byte order).

## Frame format

Request (configurator -> FC):

```
'$' 'M' '<' SIZE CMD PAYLOAD[SIZE] CHECKSUM
```

Response (FC -> configurator):

```
'$' 'M' '>' SIZE CMD PAYLOAD[SIZE] CHECKSUM
```

- `SIZE`: 1 byte, payload length (0-250)
- `CMD`: 1 byte, command ID (see below)
- `CHECKSUM`: 1 byte = `SIZE XOR CMD XOR payload[0] XOR payload[1] XOR ...`

Max payload is 250 bytes (`FRAME_MAX_PAYLOAD` in msp.cpp) - comfortably above
anything currently sent.

## Commands

| ID  | Name | Direction | Payload |
|-----|------|-----------|---------|
| 100 | IDENTIFY | request (empty) / response | ASCII string, e.g. `"ESP32FC-1.0"` (no null terminator, length = frame size) |
| 101 | STATUS | request (empty) / response | `armed:u8, failsafe:u8, mode:u8 (0=acro,1=angle,2=horizon), gyroCalibrated:u8, batteryVoltage:u16 (0.01V units), totalCurrent:u16 (0.01A units)` |
| 102 | RAW_IMU | request (empty) / response | `gyroDegS[3]:f32, accelG[3]:f32` |
| 103 | ATTITUDE | request (empty) / response | `rollDeg:f32, pitchDeg:f32, yawDeg:f32` |
| 104 | RC | request (empty) / response | `channelUs[16]:u16` (already converted to microseconds) |
| 105 | MOTOR | *(reserved, unused - see MOTOR_TEST for the actual bench-test path)* | - |
| 106 | PID | request (empty) / response | `for axis in 0..2: {P:f32, I:f32, D:f32, FF:f32}`, then `dtermLowpassHz:f32, gyroLowpassHz:f32, rpmFilterEnabled:u8, levelGainP:f32, horizonTiltEffect:f32, maxAngleDeg:f32` |
| 107 | SET_PID | request: same payload as PID response / response (empty) | writes the above into RAM (not flash - call SAVE_SETTINGS to persist) |
| 108 | RATES | request (empty) / response | `for axis in 0..2: {rcRate:f32, superRate:f32, expo:f32}` |
| 109 | SET_RATES | request: same as RATES response / response (empty) | |
| 110 | MODES | request (empty) / response | `for mode in 0..3 (ARM,ANGLE,HORIZON,BLACKBOX): {auxChannel:i8 (-1=unassigned, 0=AUX1), rangeStartUs:u16, rangeEndUs:u16}` |
| 111 | SET_MODES | request: same as MODES response / response (empty) | |
| 112 | MISC | request (empty) / response | `rxMinUs:u16, rxMidUs:u16, rxMaxUs:u16, motorIdlePercent:f32, batteryCellOverride:i8 (0=auto), batteryWarnV:f32 (per cell), batteryCritV:f32 (per cell), blackboxRateDivider:u8, motorPolePairs:u8, failsafeTimeoutMs:u32, boardAlignRollDeg:f32, boardAlignPitchDeg:f32, boardAlignYawDeg:f32, bidirDshotEnabled:u8, motorRemap[4]:u8 (motorRemap[logicalMotor]=physical output 0-3), motorDirectionReversed[4]:u8` — bidir flag applies on next boot (SET+SAVE+REBOOT). `motorRemap` is applied live. `motorDirectionReversed` is **report-only** in SET_MISC (the FC reads the 4 bytes for payload symmetry but does not reprogram any ESC from here — use command 123); send them back unchanged |
| 113 | SET_MISC | request: same as MISC response / response (empty) | |
| 114 | CALIBRATE_GYRO | request (empty) | response: `success:u8`. **Blocks the FC for up to ~12s** - keep the board still, same as any FC's gyro calibration. Gated on the accelerometer confirming stillness; `success=0` means it never saw a still-enough window in time. |
| 115 | MOTOR_TEST | request: `throttle[4]:u16 (DShot values, 0 or 48-2047), physical-output order` / response (empty) | Only takes effect while **disarmed** and no direction change is in progress; expires ~1s after the last command if not refreshed (send every ~200ms while the tab is open). Each value is **hard-clamped in firmware to 50%** (MOTOR_TEST_MAX_DSHOT) regardless of what is sent. |
| 116 | ESC_TELEMETRY | request (empty) / response | `for motor in 0..3: {tempC:u8, voltage:f32, current:f32, consumptionMah:u16, eRpm:u32, lastUpdateMs:u32, bidirErpm:u32, bidirLastUpdateMs:u32}` — the first eRpm/lastUpdateMs pair is from the telemetry wire, the bidir pair is from bidirectional DShot responses (0/0 = none received) |
| 117 | BLACKBOX_INFO | request (empty) / response | `writeOffset:u32, partitionSize:u32` |
| 118 | BLACKBOX_READ | request: `offset:u32, length:u16 (<=250)` / response: raw bytes (may be shorter than requested near partition end) | Configurator scans the returned bytes for `BlackboxFrame` records (see below) |
| 119 | BLACKBOX_ERASE | request (empty) / response (empty) | |
| 120 | SAVE_SETTINGS | request (empty) / response (empty) | persists current RAM settings to NVS flash |
| 121 | RESET_DEFAULTS | request (empty) / response (empty) | resets RAM settings to firmware defaults and saves |
| 122 | REBOOT | request (empty) | response is sent, then the FC reboots ~50ms later - the serial connection will drop |
| 123 | SET_MOTOR_DIRECTION | request: `physicalMotorIndex:u8 (0-3), reversed:u8` / response: `accepted:u8` | Sends the real DShot SPIN_DIRECTION_NORMAL/REVERSED + SAVE_SETTINGS commands to that ESC over ~24 flight-loop frames and stores the flag in `motorDirectionReversed`. `accepted=0` if the FC is armed, already mid-change, or the index is invalid. Props off, disarmed only. |

## Blackbox frame format (read back via BLACKBOX_READ)

Raw flash region, sequential fixed-size records (`sizeof(BlackboxFrame)` in
`Code/firmware/include/blackbox.h`), no filesystem:

```
magic:u32 ("BLKX" = 0x424C4B58)
timestampMs:u32
gyroDegS[3]:f32
setpointDegS[3]:f32   // this is actually raw stick command, not the post-rates setpoint - see note below
pidOutput[3]:f32
motorThrottle[4]:u16
batteryVoltage:f32
armed:u8
mode:u8
```

Note: the "setpoint" field currently logs the raw -1..1 stick command rather
than the post-rate-curve deg/s setpoint the PID loop actually used - a
simplification made when wiring up `main.cpp`. Fine for basic flight review;
if you want true post-rates setpoints in the log, thread `pidUpdate`'s
internal `setpoint[]` array out through `pid.h` and pass it into
`blackboxLogFrame` instead.

To parse: scan byte-by-byte for the magic value, then read a fixed-size
struct from that offset. A record may be torn at the very end of a
`BLACKBOX_READ` chunk - request the next chunk starting at
`floor(offset_of_last_complete_record + recordSize)`.
