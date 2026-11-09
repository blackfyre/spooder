# Controls

## Gamepad, through the bridge

Button names are Bluepad32's, in the Xbox layout.

| Input | Effect |
|---|---|
| Left stick (steer mode, default) | Y walks forward or back, X steers like a car (mirrored when reversing) |
| Left stick (crab mode) | Walk in any direction, strafing too |
| `B` | Toggle steer / crab |
| Right stick X | Turn, added to the steering in both modes |
| `Home` | Toggle Stand / Walk |
| `Y` | Toggle Sleep |
| `Back` + `Home` held 1 s | Toggle Calibrate |
| `R1` / `L1` | Next / previous gait (starts in Wave) |
| D-pad up / down | Body height |
| D-pad right / left | Speed |
| `X` | Toggle stride: dynamic / fixed |
| Both stick buttons held 1 s | Clear faults |
| `L2` held + `A` / `B` / `X` / `Y` / `R2` | Gesture: say hi / wiggle / dance / attack / ball (from Stand; dance, attack and ball have no script yet, so the robot does not play them) |

## TrimUI handheld

The handheld uses the gamepad's controls, by the TrimUI's own labels, with `Start` for `Home` and `L2` or `R2` pressed beyond half travel for the triggers. The TrimUI has no stick buttons and reserves Menu and Power, so a handheld layer adds:

| Input | Effect |
|---|---|
| `Select` + `Start` | Toggle the emergency stop |
| `Select` + `Y` held 1 s | Clear faults |
| `Select` + `X` | Switch the drive screen between its panels and the driving view (nothing sent changes) |
| `Select` + `B` | Switch the pak's [sound feedback](handheld.md#sound) off or on, kept for the next run (nothing sent changes) |
| `L2` beyond half travel + `A` / `B` / `X` / `Y` / `R2` | Gesture: say hi / wiggle / dance / attack / ball, as on the gamepad; while `L2` is held the drive screen shows the [gesture overlay](handheld.md#gestures) (nothing sent changes) |
| `Menu` | Leave: Sleep first, confirmed by telemetry (Sleep or output off) or after 3 s |
| `Power` short / held 2 s | Suspend / power off, after the same Sleep request and wait |

Select is a modifier with no function of its own: a Start, X, Y or B pressed with it acts only as the combination, and not again until released. The pak starts in Sleep, so `Y` stands the robot and `Start` walks it. There is no Calibrate on the handheld; calibration stays on the bridge's portal and the desktop calibrator.
