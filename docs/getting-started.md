# Getting started

## Toolchains

[mise](https://mise.jdx.dev) provides every toolchain. The SDKs are pinned git submodules.

```sh
mise install          # Python, PlatformIO, CMake, Ninja, Arm GNU toolchain, Go
mise run setup        # fetch the pinned submodules
```

The robot firmware needs power limits chosen for your servos and supply. Copy `firmware/servo2040/src/power_limits.example.h` to `power_limits.h` and adjust it. The servo model sets how fast the joints may turn: `SPOODER_SERVO_PROFILE` names it (default `MG996R`; the table is in `core/movement/servo_profile.h`).

## Try it without hardware

```sh
mise run desktop-sim
```

This builds the virtual robot and the calibrator, runs the robot in your terminal and opens the calibrator on its port. Click **Connect**, then **Calibrate**. In the terminal, `supply 7.5`, `estop on`, `load 5`, `unplug` or `bridge gamepad` exercise the faults and the arbitration. `quit` or closing the window ends both.

The calibrator needs cgo and the Wayland, X11 and EGL development packages on Linux. `mise run desktop-deps` names any that are missing. `mise run desktop-install` adds it to your desktop's launcher with its icon.

The handheld runs without hardware too: `build/virtual-robot/virtual-robot --wifi-bridge` runs a virtual bridge in WiFi drive, and `mise run handheld-build` gives a desktop build that drives it with `--addr 127.0.0.1:52040` (see [the handheld controller](handheld.md)).

## Flash the boards

```sh
mise run build && mise run upload      # robot: copy the .uf2 to the Servo 2040 in BOOTSEL mode
mise run build-bridge                  # bridge: build...
mise run upload-bridge                 # ...and flash (BRIDGE_PORT, default /dev/ttyUSB0)
mise run monitor                       # the robot's USB console; --frames decodes the link
```

Flash both boards together when the link protocol changes. A mismatched pair fails safe: each side discards the other's frames, and the robot sleeps.

## Bench simulation

Bench simulation drives the real robot board end to end without its servo supply: gait, kinematics, telemetry, protection, the e-stop and the fault clear run on a modelled supply and servos, while the servo output stays forced off until restart.

1. Remove the servo supply, and leave the board for about 6 s so the supply sense drains.
2. Connect the Servo 2040 to a computer over USB.
3. Start or reset the board.
4. Watch LED 6: it blinks amber slowly until the computer has enumerated the board (about 1 s), then fast white until 3 s after start-up. Press USER once while it blinks white; a press while it is amber is refused.

LED 6 then shows violet for half of each second. A refusal shows on LED 6 for 3 s: red and amber means a supply was measured, red and blue means no USB host has enumerated the board. Restart the board to leave.

Telemetry flags the simulation, and the calibrator, the bridge screen and the handheld show `SIM`. If the servo supply is connected while simulating, the simulation supply fault latches and only a restart clears it. The [`spooder-hardware-check`](../.claude/skills/spooder-hardware-check/SKILL.md) skill has the details.

## Wiring the link

| TTGO T-Display | Servo 2040 (J6) |
|---|---|
| GPIO27 (TX) | J6 pin 13, GPIO21 (RX) |
| GPIO26 (RX) | J6 pin 14, GPIO20 (TX) |
| GND | J6 pin 16 (logic GND) |

3.3 V logic only, and no 5 V on these pins. The bridge's right button (GPIO35) toggles the emergency stop. For WiFi drive, wire a latching switch between the bridge's GPIO32 and GND (no resistor: the bridge enables the pull-up); the [hardware check](../.claude/skills/spooder-hardware-check/SKILL.md) covers joining the network and the probe's checks.
