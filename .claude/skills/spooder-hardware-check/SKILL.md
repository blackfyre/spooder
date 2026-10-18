---
name: spooder-hardware-check
description: Workflow for wiring, powering, flashing and bringing up the Spooder firmwares on the Pimoroni Servo 2040 and the TTGO T-Display gamepad bridge — power and ground rules, J6 header and link wiring, the staged bring-up order (USB only, bench supply, one servo, all servos, then the bridge), what the serial console, LEDs and bridge screen show, the gamepad controls, how to exercise and verify power protection and the emergency stop, the bridge's setup mode with the portal calibration workflow, and bench calibration with the desktop calibrator on USB (dialout, Linux packages, workflow). Use when a change must be confirmed on the robot or bridge, when flashing or boot fails, or when the user asks to flash, test on hardware or read a console.
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

**Bridge link wiring** (three wires, 3.3 V on both sides):

| TTGO T-Display | Servo 2040 |
|---|---|
| GPIO27 (TX) | J6 pin 13, GPIO21 (RX) |
| GPIO26 (RX) | J6 pin 14, GPIO20 (TX) |
| GND | J6 pin 16 (logic GND) |

- **Never connect the boards' 5 V rails together.** Power the bridge from its own USB or a separate 5 V supply.
- Cross TX to RX. With the bridge unplugged, the robot's RX pull-up keeps the line idle.

## Build and flash

**Robot (Servo 2040):**

```sh
# power_limits.h must exist (copy power_limits.example.h and set the limits for the fitted servos and supply)
mise run build            # commands from the bridge link (default)
# put the board in BOOTSEL: hold BOOT/USER, tap RESET; the RPI-RP2 drive must be mounted
mise run upload           # rebuilds, then copies the .uf2
mise run monitor          # USB serial; Ctrl+C to exit

mise run upload --serial  # the USB serial keys instead of the link
```

- The default build ignores serial keys and drives the robot from the bridge or a USB host speaking the link protocol. For movement bring-up without either, use `mise run upload --serial`. `upload` rebuilds first and passes the flag on, so give it to `upload` itself, not only to an earlier `build`.
- In the default build, with no bridge and no USB host in control, the robot never leaves Sleep: LED 6 blinks amber fast (link stale).
- **Flash both boards together** whenever the link protocol version changes (it is 2 now): build and flash the robot and the bridge from the same commit. A mismatch shows as `Link: LOST` on the bridge and a sleeping robot. Roll back both from the previous commit.

**Bridge (TTGO T-Display):**

```sh
mise run build-bridge                         # first run downloads ESP-IDF and the toolchain
BRIDGE_PORT=/dev/ttyUSB0 mise run upload-bridge
mise run monitor-bridge                       # Bluepad32 console; type `help`
```

- Flashing overwrites whatever the TTGO runs (for example LilyGO's factory demo).
- Use a direct USB port: a hub has caused `error -71` drop-outs on this CP2104.
- If flashing fails to connect, hold GPIO0 (left button) while tapping RST.

- If `upload` reports no RPI-RP2 drive, mount it first. The task looks it up with `findmnt` by its label.
- The console takes a second or two to re-enumerate after a reset.

## What the board shows

**Serial (USB):**
- once per second: `supply <A> A (avg <A> A), <V> V; output on|off; mode <n>; faults 0x..`;
- `output allowed: starting movement` when the body starts (boot, or restart after a fault);
- `movement stopped: waiting for the fault to clear and a new request` when a fault cuts output;
- `fault latched: <name> (<A> A, <V> V)` once per fault;
- `point impossible to reach: (...) leg n` once per rejection;
- `operator source: none|bridge|USB|conflict (bridge and USB): sleeping` on each change (default build).

In the default build, a connected USB host also receives binary link frames (telemetry every 100 ms, calibration reports in Calibrate) between the text lines; a plain terminal shows them as stray characters. The text itself stays readable.

**LEDs** (`core/status/led_pattern`; LEDs 1–5 = main indication, LED 6 = command source):

| Pattern | Meaning |
|---|---|
| All six pulsing amber | Waiting for a valid supply (USB only, supply off, or settling) |
| All six alternating a colour with red, brighter | Latched fault: white overcurrent, magenta brown-out, blue overvoltage, orange sustained current, yellow undervoltage |
| LEDs 1–5 slowly breathing blue | Body stopped: supply valid, waiting for a new command (after a fault, or a Sleep request at start) |
| Green, *n* of LEDs 1–5 lit | Running; *n* = mode + 1 (1 Initialise, 2 Stand, 4 Sleep, 5 Calibrate) |
| Load bar on LEDs 1–5 in the gait's colour | Walking: lit segments = supply current against the trip limit (1.8 A each at a 9 A trip); segments past the sustained limit are red. Gait colours: Tri green, Wave sky blue, Ripple violet, Bi yellow, Quad mint, Hop pink |
| LED 6 dim white | Commands from the USB serial hook |
| All six steady red | Emergency stop latched (from the bridge) |
| LED 6 cyan / blinking magenta / fast-blinking amber | Bridge link up with gamepad / without gamepad / link stale |
| LED 6 blue | A USB host with a connected controller is in control |
| LED 6 alternating magenta and cyan | Source conflict: the bridge and a USB host both claim control; the robot sleeps until one lets go |

**Serial keys** (only in a `--serial` build):

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
   - Recalibrate the joints per the `spooder-leg-kinematics` skill (live calibration in Calibrate, then export to `core/movement/calibration_defaults.cpp`), on the bench with the desktop calibrator (below).

## Bridge bring-up

Robot on the bench supply with **no servos** until step 4; default (link) robot build.

1. **Bridge alone, on USB.** The screen shows `Pad: none` and `Link: LOST`. The console prints the Bluepad32 banner.
2. **Link on a bench cable.** Wire the three link wires.
   - Within a second the screen shows `Link: ok`, the robot's body mode, `Out on/off`, supply voltage and current. `Link: ok  USB in control` or `Link: ok  CONFLICT with USB` means a USB host claims control too.
   - LED 6 on the robot blinks magenta (link up, no gamepad).
   - Pull the cable: the screen shows `Link: LOST` within about 1 s, and the robot sleeps (LED 6 fast amber).
3. **Gamepad.** Put the pad in pairing mode (see below).
   - `Pad: connected`; the robot stands (LED 6 cyan).
   - Check every row of the controls table, including the left stick's forward direction (+Y must walk forward).
   - Switch the pad off: the robot sleeps.
4. **Emergency stop.** Press the bridge's right button (GPIO35).
   - Screen: `E-STOP`. Robot: all six LEDs steady red, output off within 20 ms (scope a servo pulse line against the bridge's TX line).
   - Press again to release: output stays off. Hold both stick buttons for 1 s to clear, then make a new mode request (`Home`) to move again.
5. **Timing under load.** With a logic analyser on the link TX, confirm a command frame at least every 20 ms while the screen changes many values.
6. **All 18 servos,** robot on a stand: walk through the controls again.

Record results in `openspec/changes/add-gamepad-bridge/tasks.md` (task 4.2).

## Setup mode and portal calibration

Setup mode turns the bridge into a WiFi access point with a captive portal; Bluetooth is off throughout, so no gamepad can connect. The link and the bridge's e-stop button work as in normal operation.

- **Enter:** power up or reset the bridge, and once the logo is up, hold the left button (GPIO0) for 1 s. Never hold it while pressing RST or plugging in: that enters the ROM download mode instead. The bridge restarts and shows `SETUP MODE`, the network name (`spooder-` and four hex digits), the password, `192.168.4.1`, `Portal: yes/no`, the link state, the e-stop and the body mode, with a joining QR code on the right.
- **Leave:** Exit on the portal, or RST. Both restart into normal operation; a power-up always starts normally.
- **Join:** scan the QR code with the phone's camera, or type the password. The password is made once and kept by the bridge. The phone's sign-in page opens the portal by itself (on Android, tap the "sign in to network" notification if it does not); otherwise browse to `http://192.168.4.1/`. One phone at a time.
- **Console:** `mise run monitor-bridge` shows `spooder bridge: setup mode (Bluetooth off)` and the access point line; the Bluepad32 console is not running.

**Calibration workflow** (robot on a stand, one servo, then one leg, then all):

1. Press **Calibrate** on the portal. The robot reports its table, and the joint editor and table fill in. Edits are refused until the first report, and the robot rejects them outside Calibrate.
2. Pick a leg and joint. Nudge the offset (±0.5°, ±1°, ±5°), the 0° and 180° pulses (±10, ±50 µs) or the direction. `Waiting for the robot...` shows until a report confirms the change; the shown values are always the robot's, and `Last change: clamped` or `rejected` says the robot changed or refused it. The joint follows at 60°/s at most.
3. **E-stop:** the bridge's right button stops output as in normal operation, and the portal shows `EMERGENCY STOP`. The button toggles: press it again to release the stop, then press **Clear faults** and a mode button to move again.
4. **Phone away:** close or lock the phone, or walk out of range. Within about 1 s the portal stops counting as a controller, and the robot sleeps. The portal's mode stays Sleep when it comes back, until a mode button is pressed.
5. **Export:** press Export, then Copy (or select the text by hand). Paste it over `core/movement/calibration_defaults.cpp` whole (leave `test/golden/` alone: its files are format fixtures), and run `mise run test`. Rebuild and flash the robot, then check in Calibrate that the robot reports the same table.

Record results in `openspec/changes/add-bridge-setup-portal/tasks.md` (task 3.2).

## Desktop calibrator (bench calibration over USB)

The desktop app in `desktop/` speaks the link protocol on the robot's USB port. It needs the default (link) robot build: in a `--serial` build the USB link is not compiled.

**Set-up (Linux):**
- Build dependencies: `mise run desktop-deps` names what is missing. On Fedora: `sudo dnf install wayland-devel libxkbcommon-devel libxkbcommon-x11-devel libX11-devel libXcursor-devel libXfixes-devel mesa-libEGL-devel vulkan-headers`.
- Port access: `sudo usermod -aG dialout $USER`, then log in again. Without it the header says `no access to /dev/ttyACM0: add yourself to the dialout group`.
- Close `mise run monitor` first: the port can be open in one program only.
- `mise run desktop-run` from the checkout. The export pane then offers that checkout (or pass `--checkout DIR` to the binary, `build/desktop/spooder-calibrator`).

**Bench workflow,** robot on a stand, bench supply, one servo first:
1. **Connect.** The Servo 2040 (USB ID 2e8a:000a) is preselected; press Connect.
   - The debug log shows the robot's text lines and the telemetry strip its mode, supply and faults. Frames go to the frames pane, never into the log.
   - A `Stray 0xA5` banner means a text line held a byte 0xA5 outside a frame: debug text must stay ASCII.
2. **Observe.** The app reports no controller: with the gamepad driving, nothing changes on the robot (LED 6 stays cyan). Space (or E-stop) still stops it: all six LEDs steady red, output off. Press Space again to release the app's e-stop; the fault stays latched.
3. **Take control** with the bridge's gamepad off and the app's e-stop released. Clear fault (the fault cannot clear while any e-stop is still signalled), then Calibrate. LED 6 shows blue (USB in control) once the fault is cleared, and the source shows `USB (this app)`.
4. **Calibrate.** Select a joint with the arrow keys or the pointer; `=`/`-` nudge fine, Shift coarse, `R` flips the direction. A change shows `pending` until the robot's report confirms it; clamped or rejected changes are marked and show the robot's value. The joint moves at 60°/s at most.
5. **Release control.** The robot sleeps (LED 6 fast amber without a bridge). Closing the window or pulling the cable does the same within about 1 s.
6. **Conflict.** Take control while the gamepad is connected: the app and the bridge screen show the conflict, LED 6 alternates magenta and cyan, and the robot sleeps until one side lets go.
7. **Export.** Show difference, check it, Write file: only `core/movement/calibration_defaults.cpp` changes. Or Copy text. Run `mise run test` (it checks the file's format and round trip, not its values), rebuild and flash, then confirm the robot reports the same table in Calibrate. Leave `test/golden/` alone: its files are format fixtures.

Record results in `openspec/changes/add-desktop-calibrator/tasks.md` (task 4.2).

## Gamepad controls

Button names are Bluepad32's (Xbox layout: `a` bottom, `b` right, `x` left, `y` top). `core/gamepad_mapping` implements this table.

| Input | Effect |
|---|---|
| Left stick | Walk direction and speed (8 % dead zone) |
| Right stick X | Rotate |
| `Home` (system) | Toggle Stand / Walk |
| `y` | Toggle Sleep (returns to the previous mode) |
| `Back` + `Home` held 1 s | Toggle Calibrate (leaving returns to Stand) |
| `r1` / `l1` | Next / previous gait |
| D-pad up / down | Body height ±10 |
| D-pad right / left | Speed ±10 |
| `x` | Toggle stride: dynamic / fixed |
| Both stick buttons held 1 s | Clear faults |
| Bridge right button (GPIO35) | Toggle emergency stop |

- One gamepad at a time; a second is refused.
- Disconnecting requests Sleep with zero input. Reconnecting starts in Stand with height, speed, gait and stride kept.
- After any fault: clear it, then make a new mode request; clearing alone never restarts motion.
- Pairing: DualShock 4 and DualSense hold Share + PS; Switch Pro holds the sync button; 8BitDo pads use their pairing combination. Bluepad32 remembers paired pads; `del_bluetooth_keys` in the bridge console forgets them.

## Report

- State what was flashed: the commit or "working tree", plus the limits file.
- List each stage and its result, and anything not checked.
- Quote serial lines rather than paraphrasing them.
- If the user did the steps, ask for their output rather than assuming it.
