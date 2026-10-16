---
name: spooder-leg-kinematics
description: Reference for Spooder's movement module — the MovementCommand interface, body modes and transitions, leg coordinate frame, link lengths, inverse kinematics and target validation, servo channel map, 50 Hz pulse output, calibration offsets, Bézier foot paths, the gait table and time-based rates. Read before changing anything in src/movement/, tuning a gait, recalibrating a leg, or feeding a controller into movement.
---

# Spooder leg kinematics and gaits

Behaviour is specified in OpenSpec (`locomotion` and `leg-actuation` capabilities; until archived, under `openspec/changes/port-upstream-locomotion/specs/`). Units are millimetres, degrees and milliseconds. The maths derives from Aecert's Hexapod V1 (upstream `ca5ed27`).

## Layout

| File | Role | Hardware-free |
|---|---|---|
| `movement.h/.cpp` | Public API: `setupMovement()`, `movementTick(nowMs, cmd)`; reports rejections, drives `servos` | no |
| `movement_command.h` | `MovementCommand`, `RequestedMode`, `StrideMode` | yes |
| `body.h/.cpp` | `BodyState` and the state machine: modes, transitions, smoothing, walking | yes |
| `gait.h/.cpp` | Gait table, progress rate, Bézier foot paths (`footTarget`) | yes |
| `kinematics.h/.cpp` | `solveLeg`, `angleToMicroseconds`, link lengths, `LEG_OFFSETS` | yes |
| `vectors.h` | `Vector2`/`Vector3`, `lerp`, `mapFloat` | yes |
| `servos.h/.cpp` | PCA9685 setup, channel map, pulse writes, output enable | no |

Hardware-free files must not include Arduino or driver headers: they build in the `native` env and are covered by `mise run test` (`test/test_*`).

## Command interface

A controller builds a `MovementCommand` each loop and hands it to `movementTick` on the main loop:

- **Fields:** `mode` (Stand/Walk/Sleep/Calibrate), `translate` (magnitude ≤ 100), `rotate` (±100), `height` (0–100; higher raises the body), `speed` (0–100), `gait` (index into `gait::Gait`), `stride` (Dynamic/Fixed) and `issuedAtMs`.
- **Clamping:** `body::clampCommand` clamps out-of-range and non-finite values. An unknown gait is ignored.
- **Authority:** movement alone changes mode, gait and leg state; a command only requests.
- **Staleness:** a command older than `COMMAND_TIMEOUT_MS` (1 s) counts as stale and puts the body to sleep.

Without a controller, `main.cpp` sends a fresh Stand command every tick. Built with `-DSPOODER_SERIAL_COMMANDS`, it uses `control/serial_commands` instead.

## Body modes

```
Initialise --> Stand <--> Walk
                 ^ \
                 |  +--> Calibrate --> (Stand)
     Sleep <-- any mode on Sleep request or stale command
     Sleep --> Stand on any other request (outputs re-enabled)
```

- **Pre-emption:** Sleep and stale commands pre-empt everything. Other requests, including waking, wait until `transitionActive()` is false (Initialise, an active Stand transition, a Sleep fold). A command stamped up to `MAX_COMMAND_LEAD_MS` (100 ms) after `nowMs` is fresh. Once a command is stale, that `issuedAtMs` stays stale (latched), so `millis()` wrap can never revive it; only a new stamp wakes the body.
- **Initialise:** holds (160,0,0) for 25 ms, then (225,0,115) for 500 ms, then enters Stand.
- **Stand:** a Bézier move from each foot's position, through a lifted midpoint, to (190, 0, ground).
  - Coming from Walk, it moves three legs at a time, highest first.
  - Coming from Initialise, Sleep or Calibrate, it moves all legs at once.
  - Once settled, the feet track height changes.
- **Time grid:** `body::step` runs the whole state machine in 5 ms sub-steps (`INTEGRATION_STEP_MS`), carrying leftover time across transitions, so behaviour does not depend on tick length. Sub-steps only record feet (cheap `isValidTarget` check); IK is solved once per step (`solveAngles`) for legs that moved.
- **Walk:** gait progress and phase tracking (`gait::trackPhase`) advance per sub-step, so a ground phase shorter than a tick is not skipped. Foot paths are evaluated once per step, after progress.
  - On a ground↔swing phase change, a leg starts its new segment from the previous segment's end point, which keeps the path independent of step size.
  - Zero input stops progress and holds every foot.
- **Sleep:** feet ease to `REST_POSE` (130,0,−46), snap within 1 mm, then `outputsEnabled` goes false and `servos` switches every channel off. Waking re-enables output and passes through Stand.
- **Calibrate:** first raises only the feet below −20 mm. It then steps every coordinate towards `CALIBRATION_POSE` (224,0,116) at 50 mm/s, from either side.

## Leg frame and IK

Each leg is solved in its own frame, with the origin at the coxa pivot, **x** pointing outward, **y** along the body and **z** up (positive). Ground height is `groundForHeight(h) = −60 − 1.7·h`.

Link lengths are coxa 41, femur 116 and tibia 183 mm; `LEG_REACH` is 340 mm.

```
theta1 = atan2(y, x)                     + offset.x   // coxa
l1     = sqrt(x² + y²) - a1
h      = sqrt(l1² + z²)
phi1   = acos((h² + a2² - a3²) / (2·h·a2))           // clamped to [-1, 1]
phi2   = atan2(z, l1)
theta2 = (phi1 + phi2)                   + offset.y   // femur
phi3   = acos((a2² + a3² - h²) / (2·a2·a3))
theta3 = 180 - phi3                      + offset.z   // tibia
```

- **Target validation:** `solveLeg` rejects non-finite targets and targets beyond reach.
  - On rejection the body keeps the leg's recorded position, writes nothing, and sets `newlyRejected` once per accepted→rejected change. `movement.cpp` prints that rejection once.
- **Pulse conversion:** angles are clamped to 0–180° and converted to 500 + angle·2000/180 µs.
- **Pulse output:** `servos` runs both PCA9685s at 50 Hz and caches the prescale. It writes counts with `setPWM` and skips channels whose count has not changed.

## Servo map and calibration

Leg index `i` maps to driver `i < 3 ? 0x40 : 0x41`, channels `(i % 3)·3 + {0,1,2}` (coxa, femur, tibia).

| Leg | Driver | Channels | `LEG_OFFSETS[i]` (coxa, femur, tibia) |
|---|---|---|---|
| 0 | `0x40` | 0 / 1 / 2 | 90, 75, −18 |
| 1 | `0x40` | 3 / 4 / 5 | 93, 75, −15 |
| 2 | `0x40` | 6 / 7 / 8 | 93, 75, −18 |
| 3 | `0x41` | 0 / 1 / 2 | 87, 80, −26 |
| 4 | `0x41` | 3 / 4 / 5 | 85, 89, −16 |
| 5 | `0x41` | 6 / 7 / 8 | 93, 85, −24 |

These offsets are upstream's 2023 values and have never been measured on this robot.

- `STRIDE_MULTIPLIER = {1,1,1,−1,−1,−1}` puts legs 0–2 on one side and 3–5 on the other.
- `ROTATION_MULTIPLIER = {−1,0,1,−1,0,1}`, combined with `LEG_PLACEMENT_ANGLE = 56°`, places legs 0/3 at one end, 1/4 in the middle and 2/5 at the other end. Confirm which end is the front on the robot.

To recalibrate, put the robot on a stand and send `c` with the serial hook to reach the calibration pose. In that pose the coxa should be perpendicular to the body, the femur horizontal and the tibia vertical. Adjust one `LEG_OFFSETS` entry at a time and update the table.

## Gait table (`gait.cpp`)

Phase offsets are in progress units out of `CYCLE_POINTS` = 1000. Legs are listed in order 0–5.

| Gait | Phase offsets | `pushFraction` | speed × | stride × | lift × | max stride | max speed |
|---|---|---|---|---|---|---|---|
| Tri | 0, ½, 0, ½, 0, ½ | 3.1/6 | 1 | 1.2 | 1.1 | 240 | 200 |
| Wave | 0, 1/6, 2/6, 5/6, 4/6, 3/6 | 5/6 | 0.4 | 2 | 1.3 | 150 | 160 |
| Ripple | 0, 4/6, 2/6, 5/6, 1/6, 3/6 | 3.2/6 | 1 | 1.3 | 1 | 220 | 200 |
| Bi | 0, ⅓, ⅔, 0, ⅓, ⅔ | 2.1/6 | 4 | 1 | 1.8 | 230 | 130 |
| Quad | 0, ⅓, ⅔, 0, ⅓, ⅔ | 4.1/6 | 1 | 1.2 | 1.8 | 220 | 200 |
| Hop | all 0 | 3/6 | 1 | 1.6 | 2.5 | 240 | 200 |

- **Progress rate:** progress advances at `max(|translate|, |rotate|) · speed× · speedScale`, clamped to `maxSpeed · speedScale`, × 50 per second. `speedScale = (speed + 10) / 100`; the rotation scale is fixed at 0.55.
- **Fixed stride:** stride and rotation length are a constant 70, and the input sets direction only.
- **Adding a gait:** add an enum member, a `GAITS[]` row and raise `GAIT_COUNT`.

## Time-based rates (`body.cpp`)

These are converted from the original per-loop values at a 20 ms reference step and are meant to be tuned on hardware:

| Constant | Value |
|---|---|
| `STAND_RATE` | 1250 units/s |
| Translate smoothing τ | 250 ms |
| Rotate smoothing τ | 160 ms |
| Height smoothing τ | 500 ms |
| Sleep fold τ | 330 ms |
| `CALIBRATION_RATE` | 50 mm/s |
| `MAX_STEP_MS` | 100 ms |

Changing them does not change the spec. Keep the tick-rate test (`test_walk_is_tick_rate_independent`, ≤ 5 mm) passing.
