# MG996R servo: reference model

A 3D model of the MG996R, the servo fitted to every joint. From [isamu-isozaki/biped-robot](https://github.com/isamu-isozaki/biped-robot) at commit `ce8cfc8` (`exports/servo motors/servo-motor-mg996r servo.step`, a FreeCAD export of 2022-07-09), copied unchanged under its MIT licence (`LICENSE`).

| File | What it is |
|---|---|
| `MG996R.step` | the servo with its mounting ears and output spline |

The model measures 53.8 mm across the mounting ears, 20.0 mm wide and 44.4 mm tall including the output spline, matching the MG996R's usual drawing (body about 40.7 × 19.7 mm, ears about 54 mm across). Clones vary by a few tenths of a millimetre: measure a fitted servo before relying on a tight fit.

Spooder's servo profile (rated speed, share usable under load) is in `core/movement/servo_profile.h`.
