---
name: spooder-hardware-check
description: Workflow for building, flashing and observing the Spooder firmware on the Arduino Nano RP2040 Connect — safety preparation, upload, serial and OLED boot signals, and what "working" looks like per subsystem. Use when a change must be confirmed on the robot, when upload or boot fails, or when the user asks to flash, test on hardware, or read the serial monitor.
---

# Spooder hardware check

A successful `mise run build` is the automated gate. This workflow is the hardware gate; it needs the board connected and, for anything that moves, a person present. Do not flash code that drives servos without telling the user first — the robot moves on boot (Initialise, then Stand, on the first movement ticks after `setup()`).

## 1. Prepare

- Robot on a stand with all six feet off the ground before any upload that touches `movement`.
- Servo power supply on (the PCA9685 boards are not powered from USB); USB connected to the Nano.
- Check the board is visible: `pio device list` should show a USB serial device (usually `/dev/ttyACM0`). On Linux, a permission error means the user is not in the `dialout` group or the PlatformIO udev rules are missing — report it; do not change system configuration yourself.

## 2. Build and flash

```sh
mise run build     # must pass first
mise run upload    # pio run --target upload
```

Upload uses the `picotool` protocol (`rp2040load`): PlatformIO opens the serial port at 1200 baud to reset the Nano into its bootloader, then flashes it. If it cannot find the board, double-tap the Nano's reset button to enter bootloader mode and retry. A successful upload ends with `[SUCCESS]`; the board then reboots and the serial port reappears.

## 3. Observe

```sh
mise run monitor   # pio device monitor, 115200 baud; Ctrl+C to exit
```

The monitor holds the port; close it before the next upload.

Expected boot sequence (serial and OLED):

| Step | Serial | OLED |
|---|---|---|
| Display | `OLED begun` | splash, then `Boot` |
| Gamepad | `Firmware version installed: …`, `BD Address: …` | `.` |
| IMU | `Activity Recognition for LSM6DSOX MLC`, `UCF Number Line=…`, `Program loaded inside the LSM6DSOX MLC`, then about 3 s pause and an `Activity: …` line | `.` |
| ToF | `Sensor found!` or `Failed to find sensor` | `.` |
| Servos | `Setting up PCA Panels`, `PCA Panels initiated, prescale N` (N ≈ 121 for 50 Hz) | `.` |
| Ready | — | `done!`, then log lines `Ready!`, `Waiting on gamepad...`; amber `LED_BUILTIN` on |
| Movement | — | legs go to `(160,0,0)`, then `(225,0,115)`, then settle into Stand over about a second; the OLED keeps updating meanwhile |

Failure signatures:

- **Stops before `OLED begun`**, or the OLED stays blank: I²C bus or display wiring; nothing after it will work.
- **Built-in LED blinking at 2 Hz after the UCF line**: the MLC program failed to load (`Error loading the Program to LSM6DSOX at line: N`). The firmware halts here by design.
- **`Failed to find sensor`**: the ToF is absent or miswired. The firmware carries on without it.
- **`Point impossible to reach: (x,y,z) leg: N`**: an IK target beyond 340 mm or non-finite. The leg keeps its last position; reported once per rejection.
- **Legs twitch or go to the wrong angles**: check the channel map and offsets in `spooder-leg-kinematics` before touching code.

## 4. Movement checks (serial hook)

Until the gamepad drives movement, build with the serial hook and send single characters from `mise run monitor`:

```sh
PLATFORMIO_BUILD_FLAGS=-DSPOODER_SERIAL_COMMANDS mise run upload
```

| Key | Command | Expect |
|---|---|---|
| `w` / `r` / `x` | walk forward / rotate / walk with no input | legs cycle in the current gait; `x` holds still |
| `g` | next gait (Tri, Wave, Ripple, Bi, Quad, Hop) | legs re-phase and keep walking |
| `f` | toggle fixed stride | stride length no longer depends on input |
| `+` / `-` | body height ±10 | body rises / lowers smoothly |
| `s` | stand | walking legs return three at a time |
| `z` | sleep | legs fold, then go limp (outputs off) |
| `c` | calibrate | low legs rise, then all go to (224,0,116) |

With the hook, a command older than 1 s never occurs (it is re-stamped every tick), so stale-command Sleep is only exercised by the host tests.

Measurements worth recording: one servo pulse at a known angle on a scope or logic analyser (500 µs at 0°, 2500 µs at 180°, ±2 %, 20 ms frame), and the loop period (toggle a pin or print `millis()` deltas temporarily).

## 5. Per-subsystem checks

- **Gamepad**: put the controller into pairing mode. Expect `CALLBACK: Controller is connected, index=0`, the blue RGB LED on and `Gamepad found!` on the OLED. Buttons light red/green and log their letter. The labels are swapped for 8BitDo pads (see the comments in `processGamepad`).
- **ToF**: move a hand under the sensor. `Range: N` lines appear on the OLED only when the value changes.
- **IMU / MLC**: move or shake the board. `Activity: …` lines appear on interrupt.
- **Display idle**: 50 s without a log line, the logo replaces the log (`IDLE_LOGO_MS`).

## 6. Report

State what was flashed (commit or "working tree"), which checks were observed and their result, and anything not checked. Quote the relevant serial lines rather than paraphrasing them. If the user ran the steps, ask them for the serial output instead of assuming the outcome.
