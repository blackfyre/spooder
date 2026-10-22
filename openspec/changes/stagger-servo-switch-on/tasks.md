# Tasks

## 1. Staggered switch-on

- [x] 1.1 In `core/robot/robot_tick`, count the legs released since output switched on (two more per tick from the tick that switches it on, none while it is off), treat unreleased legs as unknown in `limitOutput`, and pass the count to `Hardware::applyServos`. Verify with `test_robot_tick` cases: the release order across three ticks, a restart after output switched off, and a released leg's first pulse at its command.
- [x] 1.2 Honour the count in `firmware/servo2040/src/servo_output` and the virtual robot's `sim::Servos`: enable only released legs, pulse a leg's set outputs when it is released, clear on switch-off. Verify that `mise run build` with and without `--serial` and `mise run virtual-robot` pass.

## 2. Integration and documentation

- [x] 2.1 Update the `spooder-leg-kinematics` or `spooder-firmware-shape` skill where it describes switch-on, and add the `CHANGELOG.md` entry. Verify by reading them against the spec delta.
- [x] 2.2 Run the gates: `mise run test`, `mise run build` with and without `--serial`, `mise run virtual-robot`, `mise run e2e`. Verify that all of them pass.
