# Tasks

## 1. Test harness and pure core

- [x] 1.1 Add `[env:native]` (platform `native`, Unity, `test_build_src = yes`, `build_src_filter` limited to pure movement files) and a `mise` task `test` running `pio test -e native`; verify `mise run test` runs an empty placeholder test and passes, and `mise run build` still targets the board env by default
- [x] 1.2 Make `vectors.h` pure: drop the `main.h` include, move `toString()` formatting into `reporting`, add `lerp(Vector3)`, `Vector2::magnitude()` and a zero-guarded `Vector2::normalize()`; verify host tests for lerp/normalize (including zero vector) pass and the firmware build succeeds
- [x] 1.3 Create `kinematics.{h,cpp}`: `solveLeg` (IK from `moveToPos`, unchanged maths), rejection of non-finite and >340 mm targets, 0–180° clamp, angle→pulse conversion; verify host tests for the (224,0,116) calibration pose, 360 mm and NaN rejection, and clamping pass
- [x] 1.4 Create `gait.{h,cpp}`: Bézier evaluation, gait table (our enum order and current parameters), phase offsets, `getGaitPoint` ported with the zero-input guard and fixed-stride branch; verify host tests for Bézier end points, phase/re-phase per gait, zero-input and rotate-only finiteness, and fixed stride length independent of magnitude

## 2. Body state machine

- [x] 2.1 Create `body.{h,cpp}` with `BodyState`, `MovementCommand` parsing/clamping and the single transition function (Sleep/stale pre-empt; other requests wait for transitions); verify host tests for clamping, unknown-gait ignore, and Walk-request-during-Stand ordering
- [x] 2.2 Port Initialise as timed sub-steps and Stand as a non-blocking transition (≤3 legs from Walk, all legs from Initialise/Sleep/Calibrate, height tracking while standing); verify host tests for the three-leg limit and that no call loops internally
- [x] 2.3 Port Walk with dt-scaled cycle progress and gait change re-phasing; verify the tick-rate independence test (20 ms vs 50 ms steps, ≤5 mm divergence) passes
- [x] 2.4 Add command smoothing (τ translate 250 ms, rotate 160 ms, height 500 ms) with reset on Sleep entry; verify host tests that a 0→100 step is eased and that Sleep zeroes the applied values
- [x] 2.5 Add Sleep (fold to (130,0,−46), snap within 1 mm, then request outputs off; wake re-enables and passes through Stand) and stale-command handling (1 s timeout); verify host tests for sleep entry order, wake ordering and stale → Sleep from Walk
- [x] 2.6 Port Calibrate with upstream's "raise only low legs" phase and a signed bounded-speed approach to (224,0,116); verify host tests for the low-legs-only phase and approach from above without jumping

## 3. Hardware layer and integration

- [x] 3.1 Create `servos.{h,cpp}`: `Leg legs[6]` table holding driver pointers, 50 Hz `setPWMFreq` on both drivers with cached prescale, pulse writes via `setPWM` with unchanged-channel skipping, output enable/disable via full-off; verify the firmware build succeeds and no `Leg` copies remain (`grep` for by-value leg arrays)
- [x] 3.2 Create the `movement.{h,cpp}` public API (`setupMovement`, `movementTick(nowMs, cmd)`, `MovementCommand`), wiring body → kinematics → servos with record-only-on-accept and once-per-transition rejection reporting; remove the old `movement.cpp` globals and state functions; verify firmware build succeeds and all host tests pass
- [x] 3.3 Update `main.cpp`: `Wire.setClock(400000)`, call `setupMovement()`, replace `delay(50)` with a 20 ms period scheduler calling `movementTick` with a fresh default Stand command each tick; verify the firmware build succeeds
- [x] 3.4 Fix the display bus budget: remove the duplicate `display.display()` and redraw only on log change, at most every 200 ms; verify the firmware build succeeds
- [x] 3.5 Add a temporary serial command hook behind a `SPOODER_SERIAL_COMMANDS` build flag (single characters selecting Stand, Walk forward, Walk rotate, Sleep, Calibrate) for hardware checks before the controller exists; verify the build succeeds with and without the flag

## 4. Verification

- [x] 4.1 Run `mise run build` and `mise run test`; verify both pass with no new warnings in `src/`
- [ ] 4.2 Hardware check per `spooder-hardware-check`, with the robot on a stand (requires the user at the robot): boot runs Initialise then Stand without stalling the OLED; loop period and one servo pulse width measured (2450–2550 µs at 2500 µs requested); serial hook exercises Walk, gait change, Sleep (servos go limp) and wake, Calibrate; record results in the change
- [x] 4.3 Update `.claude/skills/spooder-leg-kinematics` and `spooder-firmware-shape` to the new module layout, API and rates; verify the skills reference only symbols that exist (`grep`)
