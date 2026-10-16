# Proposal

## Why

Spooder's movement code was copied from Aecert's Hexapod V1 (upstream `ca5ed27`, April 2024), but the robot cannot walk. Nothing calls the gait or stand states, the walking maths produces NaN on some inputs, and the stand transition blocks the main loop for about a second. Upstream has since added movement features we want (sleep with limp servos, a fixed-stride mode, corrected calibration motion). We want those without inheriting upstream's radio-controller logic, so our own gamepad controller can drive movement through a clean command interface.

## What Changes

- Add a **movement command interface**: a plain `MovementCommand` value (requested mode, translate vector, rotate amount, body height, speed scale, gait, stride mode, timestamp) that a controller hands to movement. Movement owns every state and gait transition; the controller never sets movement state directly.
- Replace the blocking state functions with a **non-blocking `movementTick(dt, command)`** called once per main-loop iteration. Every transition (Initialise, Stand, Walk, Sleep, Calibrate) advances by elapsed time and returns.
- Port upstream's **Sleep mode**: legs fold to a rest pose, then all servo outputs switch off. The next motion wakes the outputs.
- Port **command smoothing**: translate and rotate inputs ease towards their targets, scaled by elapsed time instead of upstream's per-tick lerp.
- Port **fixed-stride mode** (constant stride and rotation length regardless of input magnitude) and make body height and speed scale command parameters instead of hard-coded values.
- Port upstream's **calibration motion fix** (only lower legs that are still low) and fix the stepping bug present in both versions, which snaps to the target when approaching from above.
- **Stale command → Sleep**: if no fresh command arrives within a timeout, movement goes to Sleep.
- Port helpers `lerp(Vector3)`, `Vector2::magnitude()` and `Vector2::normalize()`.
- Fix our own defects: unreachable IK targets are no longer recorded as the leg's position; zero input no longer produces NaN in the gait maths; servo writes no longer copy all six leg structs on every call.
- Gaits keep **our** enum order and parameters; upstream's re-tuning for its new feet, its link lengths and its offsets are not ported.

Out of scope: gamepad mapping and mode-selection logic (our own controller will be specified separately), upstream's radio protocol and telemetry, runtime offset editing and persistence, the slam attack (deferred), and gait re-tuning on our hardware.

## Capabilities

### New Capabilities

- `locomotion`: Body modes (Initialise, Stand, Walk, Sleep, Calibrate), the movement command interface, non-blocking time-based transitions, gait selection and phasing, smoothing, stride mode, and stale-command handling.
- `leg-actuation`: Per-leg inverse kinematics and its reachability rule, joint-angle-to-servo output on the two PCA9685 drivers, and enabling/disabling servo output.

### Modified Capabilities

None. There are no existing specs.

## Impact

- **Code:**
  - `src/movement/` is restructured (`movement.cpp`, `movement.h`, `vectors.h`). A new public header type `MovementCommand` is added.
  - `src/main.cpp` calls `movementTick` from `loop()` in place of nothing.
  - `src/control/` is not changed by this change; until the controller change lands, movement receives a default command that holds Stand.
- **Concurrency:** movement runs only on the main loop, which already owns the I²C bus. The command is passed by value, so no new shared state is introduced in this change.
- **Hardware:** behaviour on the robot changes at boot (Initialise then Stand instead of a fixed pose). Servo outputs can now be switched off.
- **Tests:** introduces a PlatformIO `native` test environment with Unity for the pure maths (IK, Bézier, gait phase, smoothing, transitions).
- **Dependencies:** none added.
