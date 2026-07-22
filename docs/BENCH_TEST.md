# Bench-test checklist - read this before your first flight

This is new firmware, on a hobby-grade gyro, that has never physically flown.
"Compiles with zero errors/warnings" means the code is internally consistent
and type/logic-correct as far as static analysis and reasoning can tell you -
it says nothing about whether the PID tune is right, whether a wire is
crossed, or whether a specific ESC/motor combination behaves the way the
firmware assumes. Work through this list in order, on a bench, before it
ever sees a battery outdoors.

## 1. Props off. All of it. Every step below.

Do not put propellers on until you've completed every step in this document
and you're standing in the field ready to fly. No exceptions for "just a
quick check."

## 2. Power discipline

- Never connect USB and the flight battery at the same time (see
  `docs/HARDWARE.md` - the XIAO's 5V pad has no reverse-protection diode from
  the factory). Bench-test over USB with the battery disconnected.

## 3. Flash and connect

```
cd firmware && pio run -t upload
```

Then open the configurator (`cd configurator && python -m http.server 8000`,
open `http://localhost:8000` in Chrome/Edge) and click **Connect**.

## 4. Setup tab: verify sensor sanity before anything else

- Lay the board flat. Roll/pitch should read close to 0 deg. Accel Z should
  read close to 1.0g, X/Y close to 0.
- Tilt the board by hand on each axis and confirm roll/pitch respond in the
  direction you'd expect (not inverted, not swapped).
- Run **Calibrate Gyro**. It should succeed within a couple of seconds if the
  board is genuinely still. If it reports failure, the board moved - find a
  more stable surface and retry. Do not proceed until this succeeds.
- With the board still, gyro X/Y/Z should read close to 0 deg/s (not
  drifting steadily in one direction - a few tenths of a degree of noise is
  normal, a steady climb is not).

## 5. Receiver tab: verify every channel before touching Modes

- Move each stick/switch on your transmitter and confirm the corresponding
  channel bar moves the way you expect, full range, no channel stuck.
- Confirm throttle reads near minimum (close to 988us) at stick-down.
- Turn your transmitter off. Confirm the FC's failsafe badge lights up
  within your configured timeout, and stays off channels don't just freeze
  at their last value.

## 6. Modes tab: set up ARM deliberately, verify it, then verify disarm

- Assign an AUX channel to ARM. Pick a switch you will never bump by
  accident.
- With the FC connected and props still off, flip the switch and confirm the
  Setup tab's status shows armed - **only** when throttle is also at
  minimum. It should refuse to arm at any other throttle position.
- Flip the switch back and confirm it disarms immediately.
- Turn off your transmitter while (harmlessly, props off) armed on the
  bench, and confirm it disarms via failsafe.

## 7. Motors tab: only after everything above passes

- **Confirm props are physically off the frame.** Look at all four motors
  with your own eyes.
- Enable motor test, spin each motor individually at low throttle, confirm:
  - It's the physical motor you expected for that slider (M1-M4 per
    `docs/HARDWARE.md`'s mixer table) - reassign wiring or accept the mapping
    and remember it, your choice.
  - Rotation direction is correct for a standard QuadX layout (diagonal
    pairs spin the same direction, adjacent motors spin opposite). Wrong
    direction means swap two of that motor's three wires at the ESC, or set
    direction via your AM32 configurator/passthrough (this firmware doesn't
    do it in-flight, see the README's known gaps).
  - ESC Telemetry tab shows sane voltage/current/temp/eRPM for each motor
    while spinning - if a motor shows no telemetry, check that wire's solder
    joint before flying.

## 8. PID/Rates: start from the shipped defaults, don't guess big changes

The defaults in `firmware/src/settings.cpp` are deliberately conservative
starting points, not a tuned result - expect to do real tuning, the same as
setting up any fresh flight controller. Change one thing at a time, retest
on the bench (props off, hand-hold the quad, watch how it responds to
gentle tilts) before ever adding propellers.

## 9. First flight

- Do it somewhere open, away from people, with the failsafe/disarm behavior
  already verified per step 6.
- Keep initial flights short and low. If anything feels wrong - oscillation,
  drift, unexpected direction on a stick - land immediately and go back to
  the bench, don't try to fly through it.
