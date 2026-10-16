# Proposal

## Why

The Nano RP2040 Connect and PCA9685 robot has been dismantled, and the next build starts from scratch. Hardware research found that the old platform needed per-chip oscillator calibration for the PCA9685s. It also had a crowded shared I²C bus, used generic soft-float under the mbed core (the RP2040's ROM float routines are not linked), and depended on Bluepad32's NINA target, which upstream has now removed. The Pimoroni Servo 2040 drives all 18 servos directly from crystal-timed PIO and can measure its own servo supply current and voltage, so the robot can protect itself from damage. This change moves the movement firmware onto the Servo 2040 and adds that protection. The Bluetooth bridge and the sensors follow in separate changes (`add-gamepad-bridge`, `add-body-sensing`).

## What Changes

- **BREAKING:** Remove the Nano RP2040 Connect firmware: the mbed/Arduino `src/` tree, the PCA9685, NINA/Bluepad32, SH1107, IKS01A3 MLC and VL6180X Arduino drivers, and the `nanorp2040connect` PlatformIO environment.
- Add a pico-sdk C++ firmware project for the Servo 2040, built with CMake and Ninja. It pins pico-sdk 2.2.0 and pimoroni-pico v1.29.0-2 as git submodules, and its toolchain comes from mise.
- Move the hardware-free movement core (`body`, `gait`, `kinematics`, `vectors`, `movement_command.h`) unchanged into a shared `core/` directory. Its host tests keep running with `mise run test`.
- Drive the 18 servos with pimoroni-pico `ServoCluster` on GPIO0–17, replacing the PCA9685 output layer.
- Add **power protection**. The firmware samples the servo supply current and voltage, holds servo output off until the supply is in range, and stops the robot safely on overcurrent, sustained high current, undervoltage or overvoltage. A fault latches until it is cleared and is shown on the board's RGB LEDs.
- Port the serial hardware-check hook to USB serial (pico stdio). It becomes the bring-up input until the gamepad bridge exists, and it also prints power readings and faults.
- Update `CLAUDE.md`, `mise.toml` and the project skills for the new platform.

## Capabilities

### New Capabilities

- `power-protection`: Monitoring of the servo supply current and voltage, the protective responses to out-of-range conditions, fault latching and clearing, and fault indication.

### Modified Capabilities

- `leg-actuation`: Servo output moves from two I²C PCA9685 drivers to the Servo 2040's 18 PIO-driven outputs. The channel mapping and frame-rate requirements change. Output enabling gains a protection inhibit that blocks the implicit wake while a fault is latched.

## Impact

- **Code:** `src/` is removed. A new `firmware/servo2040/` (CMake) contains `main`, the servo adapter, power protection and serial commands. The movement core moves to `core/movement/`. `platformio.ini` keeps only the `native` test environment.
- **Toolchain:** adds `cmake`, `ninja` and `gcc-arm-none-eabi` to mise, plus two git submodules. The firmware no longer uses the PlatformIO RP2040 toolchain.
- **Hardware (build notes, not firmware):**
  - The servo supply enters through the screw terminals, rated 10 A continuous.
  - For supplies above 5 V, the board's "Separate USB and Ext. Power" trace must be cut and logic powered separately.
  - Sensors and the bridge connect through a soldered J6 header.
- **Behaviour:** on boot, servo output stays off until the supply voltage is valid. Protection can force Sleep or cut output at any time.
- **Out of scope:** the gamepad bridge and link protocol (`add-gamepad-bridge`), the VL6180X and LSM6DSO (`add-body-sensing`), gait tuning, and measuring the new frame's geometry and offsets.
