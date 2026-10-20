# Tasks

## 1. Robot

- [x] 1.1 Add the bridge link fields to telemetry (D1) and fill them in `robot_tick`; host tests (link up/down, gamepad, counts, serial build zeros); regenerate the golden vectors deliberately
- [x] 1.2 Verify `mise run test`, both robot builds and `build-bridge`

## 2. Front-ends

- [x] 2.1 Go protocol fields against the golden vectors; console line; desktop Bridge cell (D2); tests
- [x] 2.2 Verify `desktop-test`, `desktop-build` and `e2e` (virtual robot with its simulated bridge)

## 3. Documentation and hardware

- [x] 3.1 Update the skills (firmware-shape telemetry bytes, hardware-check bring-up), CLAUDE.md if needed, and the changelog
- [ ] 3.2 On the robot: with the bridge running and no gamepad, the calibrator shows the bridge up; unplugging it shows it down
