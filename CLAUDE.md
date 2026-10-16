# CLAUDE.md

Spooder is Arduino firmware for a hexapod robot, derived from [Aecert's Hexapod V1](https://github.com/Ryan-Mirch/Aecerts_Hexapod_V1). It targets the Arduino Nano RP2040 Connect (`raspberrypi` platform, mbed-based Arduino core) and is built with PlatformIO.

## Toolchain

Tools are managed by mise (`mise.toml`): Python and PlatformIO Core (`pio`, installed via the `pipx` backend).

```sh
mise install          # install Python + PlatformIO
pio pkg install       # install the platform, toolchain and lib_deps from platformio.ini
```

## Commands

```sh
mise run build        # pio run
mise run upload       # pio run --target upload
mise run monitor      # pio device monitor (115200 baud)
mise run clean        # pio run --target clean
mise run test         # pio test -e native (host-side Unity tests for the movement core)
```

`mise run build` and `mise run test` are the verification gates for changes. Host tests cover the hardware-free movement files only; hardware behaviour can only be checked on the board.

## Architecture

- `src/main.cpp` — `setup()` initialises each subsystem in order (display, gamepad, MLC, ToF, movement), then starts a second loop with `Scheduler.startLoop(loop2)` (a pre-emptive RTOS thread).
  - `loop()` — on a 20 ms period: `movementTick()`, `loopMLC()`, `loopTOF()`, `loopDisplay()`. It owns the I²C bus (400 kHz).
  - `loop2()` — control: `loopGamepad()` (SPI to the NINA module).
- `src/control/` — Bluetooth gamepad input via Bluepad32 (through the NINA-W102 co-processor; the RGB LED is also driven via `BP32.pinMode`/`BP32.digitalWrite`). Not yet connected to movement; `serial_commands` (`-DSPOODER_SERIAL_COMMANDS`) is a temporary hardware-check input.
- `src/movement/` — `movementTick(nowMs, MovementCommand)` drives a non-blocking body state machine (Initialise, Stand, Walk, Sleep, Calibrate), gaits, IK and 50 Hz servo output on two PCA9685s. Hardware-free core: `body`, `gait`, `kinematics`, `vectors`; hardware: `servos`.
- `src/mlc/` — LSM6DSOX machine-learning core (activity recognition) on the X-NUCLEO-IKS01A3 shield; `activity_recognition.h` is the generated MLC program.
- `src/tof/` — VL6180X time-of-flight distance sensor.
- `src/display/` — SH110X OLED output and an on-screen log buffer (`addToLogBuffer`).
- `src/reporting/` — serial `print_value` helpers.
- `src/globals/wiring.h` — shared `TwoWire *globalWire`.

Each module exposes `setupX()` / `loopX()` functions declared in its header; follow that pattern for new subsystems.

## Notes

- The Bluepad32 dependency is published in the PlatformIO registry as `ricardoquesada/Bluepad32 for NINA-W10 boards`; the short name `ricardoquesada/Bluepad32` does not resolve.
- Build output and installed libraries live in `.pio/` (git-ignored).
- Intended behaviour is specified with OpenSpec in `openspec/` (`/opsx:*` commands in `.claude/`).
- A CodeGraph index lives in `.codegraph/` (git-ignored); refresh it with `codegraph sync` after large changes.
- Project skills in `.claude/skills/`: `spooder-firmware-shape` (module, concurrency and bus rules), `spooder-leg-kinematics` (movement reference), `spooder-hardware-check` (flash and verify on the board).
