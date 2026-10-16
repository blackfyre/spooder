---
name: spooder-hardware-check
description: Workflow for wiring, powering, flashing and bringing up the Spooder firmware on the Pimoroni Servo 2040 — power and ground rules, J6 header wiring, the staged bring-up order (USB only, bench supply, one servo, all servos), what the serial console and LEDs show, and how to exercise and verify power protection. Use when a change must be confirmed on the robot, when flashing or boot fails, or when the user asks to flash, test on hardware or read the serial console.
---

# Spooder hardware check

`mise run test` and the firmware build are the automated gates. This workflow is the hardware gate.

- It needs the board connected and, for anything that moves, a person present.
- Tell the user before flashing anything that drives servos. Once the supply is accepted, the robot starts moving.
- Record results in the OpenSpec change being verified.

## Power and wiring rules (read first)

- **Servo supply:** into the screw terminals, rated 10 A continuous, reverse-polarity protected, servos up to 11 V.
  - With the board's "Separate USB and Ext. Power" trace intact, the terminal supply also feeds USB VBUS and J6 pin 17 (5V).
  - For any supply above 5 V, **cut that trace**, then power the logic from USB or a separate 5 V BEC on J6 pin 17.
- **Ground:** connect external logic ground (bridge, sensors) only to **logic GND**, on J6 pins 1, 3, 5, 11 or 16 or the Qw/ST GND.
  - **Never** use the servo-connector ground pins. The current shunt sits in the servo ground return, so current returned through it is mis-measured.
- **3.3 V logic only on J6.** No pin is 5 V tolerant.
- **Pull-ups:** the board has none on GPIO19–21 or 26–28. I²C sensors need breakouts with pull-ups, or 4.7 kΩ resistors to 3V3.
- **Bench supplies:** use a current-limited bench supply for first power-up and protection tests. Set the limit below the firmware's trip current.

J6 (unpopulated 17-pin header), pin → signal: 1 GND, 2 USER_SW, 3 GND, 4 RESET, 5 GND, 6 SWDIO, 7 SWCLK, 8 GPIO28, 9 GPIO27, 10 GPIO26, 11 GND, 12 GPIO19, 13 GPIO21, 14 GPIO20, 15 3V3, 16 GND, 17 5V. Allocation: see `spooder-firmware-shape`.

## Build and flash

```sh
# power_limits.h must exist (copy power_limits.example.h and set the limits for the fitted servos and supply)
mise run build
# put the board in BOOTSEL: hold BOOT/USER, tap RESET; the RPI-RP2 drive must be mounted
mise run upload
mise run monitor          # USB serial; Ctrl+C to exit
```

- If `upload` reports no RPI-RP2 drive, mount it first. The task looks it up with `findmnt` by its label.
- The console takes a second or two to re-enumerate after a reset.

## What the board shows

**Serial (USB):**
- once per second: `supply <A> A (avg <A> A), <V> V; output on|off; mode <n>; faults 0x..`;
- `output allowed: starting movement` when the body starts (boot, or restart after a fault);
- `movement stopped: waiting for the fault to clear and a new request` when a fault cuts output;
- `fault latched: <name> (<A> A, <V> V)` once per fault;
- `point impossible to reach: (...) leg n` once per rejection.

**LEDs:**

| Pattern | Meaning |
|---|---|
| Pulsing amber | Waiting for a valid supply (USB only, supply off, or settling) |
| Green, *n* LEDs lit | Healthy; *n* = mode + 1 (1 Initialise, 2 Stand, 3 Walk, 4 Sleep, 5 Calibrate) |
| White ↔ red | Overcurrent |
| Magenta ↔ red | Brown-out |
| Blue ↔ red | Overvoltage |
| Orange ↔ red | Sustained current |
| Yellow ↔ red | Undervoltage |

**Serial keys:**

| Key | Action |
|---|---|
| `s` | Stand |
| `w` | Walk forward |
| `r` | Walk rotating |
| `x` | Walk with no input |
| `g` | Next gait |
| `f` | Toggle fixed stride |
| `z` | Sleep |
| `c` | Calibrate |
| `+` / `-` | Raise or lower body height |
| `!` | Clear faults |
| `p` | Print readings now |

## Staged bring-up

Stop at the first stage that misbehaves.

1. **USB only, no servo supply.**
   - Expect pulsing amber, output off, and a voltage near 0 V with current near 0 A.
   - Expect no faults: an absent supply isn't a fault.
2. **Bench supply, no servos, current-limited.**
   - Raise it into the configured voltage window.
   - Within 500 ms, expect `output allowed: starting movement`, and the LEDs turn green.
   - Check the reported voltage against a meter (±2 %).
3. **Overvoltage and undervoltage.**
   - With no servos: raise the supply above `SPOODER_MAX_V`. Expect an overvoltage fault immediately, with output off. Restore, then `!` clears it.
   - Lower it below `SPOODER_MIN_V` for longer than the debounce time. Expect an undervoltage fault and Sleep.
4. **One servo, on output 1.**
   - Check a 1500 µs pulse on a scope or logic analyser: ±2 %, 20 ms period.
   - Unpowered outputs show no pulse.
5. **Overcurrent.**
   - Stall the servo by hand, briefly, or set the bench limit to just above the trip current.
   - Expect output off within 20 ms, and the overcurrent LEDs.
   - Clear with `!` once the current is normal. Nothing moves until you also give a new command (for example `s`), and the supply has been valid for 500 ms. The body then restarts from Initialise.
6. **One leg, then all 18,** with the robot on a stand, feet off the ground.
   - Boot runs Initialise, then Stand.
   - Then `w`, `g`, `r`, `z` (legs fold, then go limp), `s` (wakes through Stand) and `c`.
7. **Geometry and calibration.**
   - Measure the new frame's link lengths, and compare them with `kinematics::COXA/FEMUR/TIBIA_LENGTH`.
   - Recalibrate `LEG_OFFSETS` per the `spooder-leg-kinematics` skill.

## Report

- State what was flashed: the commit or "working tree", plus the limits file.
- List each stage and its result, and anything not checked.
- Quote serial lines rather than paraphrasing them.
- If the user did the steps, ask for their output rather than assuming it.
