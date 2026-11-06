# Proposal

## Why

Driving Spooder end to end today needs the servo supply connected, so every bench session with the desktop calibrator, the bridge or a future handheld risks the legs moving. The virtual robot runs the shared control tick on a computer, but not the real board's links, timing, LEDs or USB port. A bench simulation mode on the real Servo 2040 lets the robot be driven with USB only (gait, kinematics, pose frames, telemetry, protection, the e-stop and fault clear all behaving as on a powered robot) while no servo ever moves.

## What Changes

- **Bench simulation, a runtime mode of the normal robot firmware.** For about 3 s after start-up the status LEDs show that simulation is available, while live mode runs from the first tick. A press of the board's USER button (GPIO23) in that window enters simulation only if all of these hold: the button was seen released and then pressed (an edge); the real servo supply has read below about 1 V continuously since boot; servo output is off; and a USB host has enumerated the board. A refused press shows the reason on the LEDs. Simulation lasts until the next restart; it is never persisted.
- **No servo pulse in simulation, by construction.** At the board boundary, simulation forces servo output off and routes the tick's servo calls to a servo model instead; the real servo output can no longer be switched on until restart.
- **Modelled supply.** In simulation the readings fed to protection come from the virtual robot's supply and current model, moved into `core/` and shared by the firmware and the virtual robot, so protection runs normally on them. The real supply keeps being read: if it rises above the brown-out voltage, a new fault, the simulation supply fault, latches and cannot be cleared until restart.
- **Reporting.** Telemetry gains a flag, `TELEMETRY_SIMULATED` (bit 5 of the flags byte). In simulation `outputsOn` and the pose frame report the modelled output, and the flag carries the truth. The bridge screen and the desktop calibrator show a clear SIM indication and name the new fault. No protocol version change: every receiver ignores unknown flag and fault bits.

## Capabilities

### New Capabilities

- `bench-simulation`: entry window and gates, refusal indication, duration, servo output forced off, the modelled supply, the simulation supply fault, and simulation in telemetry and pose frames.

### Modified Capabilities

- `status-display`: the bridge screen shows simulation and names the simulation supply fault.
- `desktop-calibrator`: the app shows simulation and names the simulation supply fault.

## Impact

- `core/`: a new bench simulation module (the entry decision and the board-boundary routing) and the supply and servo model moved from `tools/virtual-robot/src/sim.{h,cpp}`; `core/robot/robot_tick` (the simulation inputs, the flag, the LED indication), `core/protection` (the simulation supply fault), `core/status/led_pattern`, `core/status/status_screen`; host tests.
- `core/protocol`: the `TELEMETRY_SIMULATED` flag and a golden telemetry vector in `test/golden/protocol_v3.txt`; the wire format and `protocol::VERSION` (3) are unchanged.
- `firmware/servo2040/`: the USER button, USB enumeration state, the board boundary in `main.cpp`, and a sticky forced-off state in `servo_output`.
- `desktop/`: the flag and fault names, a SIM indication.
- `tools/virtual-robot`: uses the shared model; it reports the flag clear.
- Docs: `firmware/servo2040/CLAUDE.md`, `core/CLAUDE.md`, the `spooder-hardware-check` and `spooder-firmware-shape` skills, readme, changelog.
- Not here: the handheld pak's SIM indication (`desktop/handheld` is on the unmerged `deliver/add-handheld-controller` branch) and the setup portal's.
