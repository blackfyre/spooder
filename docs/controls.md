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
| `R1` / `L1` | Next / previous gait |
| D-pad up / down | Body height |
| D-pad right / left | Speed |
| `X` | Toggle stride: dynamic / fixed |
| Both stick buttons held 1 s | Clear faults |
| `L2` held + `A` / `B` | Gesture: say hi / wiggle (from Stand) |

## TrimUI handheld

The handheld uses the gamepad's controls, by the TrimUI's own labels, with `Start` for `Home` and `L2` or `R2` pressed beyond half travel for the triggers. The TrimUI has no stick buttons and reserves Menu and Power, so a handheld layer adds:

| Input | Effect |
|---|---|
| `Select` + `Start` | Toggle the emergency stop |
| `Select` + `Y` held 1 s | Clear faults |
| `Select` + `X` | Switch the drive screen between its panels and the driving view (nothing sent changes) |
| `L2` beyond half travel + a face button | Gesture |
| `Menu` | Leave: Sleep first, confirmed by telemetry (Sleep or output off) or after 3 s |
| `Power` short / held 2 s | Suspend / power off, after the same Sleep request and wait |

Select is a modifier with no function of its own: a Start, X or Y pressed with it acts only as the combination, and not again until released. The pak starts in Sleep, so `Y` stands the robot and `Start` walks it. There is no Calibrate on the handheld; calibration stays on the bridge's portal and the desktop calibrator.
