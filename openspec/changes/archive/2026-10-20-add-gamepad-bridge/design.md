# Design

## Context

- **Depends on `port-movement-to-servo2040`:**
  - the Servo 2040 runs movement on core 0 in a 20 ms loop;
  - `core/protection` latches faults and turns them into `forceSleep` and `allowOutput`;
  - GPIO20/21 on J6 are reserved for this link (UART1 TX/RX), with no board pull-ups.
- **Bluepad32 on the original ESP32** (Bluepad32 FAQ and programmer's guide):
  - Bluetooth Classic is supported only on the original ESP32, which the TTGO T-Display v1.1 uses.
  - The BTstack task runs on core 0, and Arduino `setup()`/`loop()` run on core 1.
  - `BP32.update()` is called once per `loop()`.
  - The Bluepad32 console occupies UART0 (GPIO1/3, the USB bridge).
- **TTGO T-Display v1.1** (LilyGO repository):
  - ST7789V 135×240 display on SPI: MOSI 19, SCLK 18, CS 5, DC 16, RST 23, backlight 4;
  - buttons on GPIO0 (a strapping pin) and GPIO35 (input only);
  - battery ADC on 34, enabled through 14;
  - free header pins: 25, 26, 27, 32, 33 and others.
- **Display library risk:** LilyGO's bundled TFT_eSPI supports arduino-esp32 only up to 2.0.14. The Bluepad32 template uses Arduino core 3.x on ESP-IDF 5.5.

## Goals / Non-Goals

**Goals:**
- One host-tested protocol implementation, shared by both firmwares.
- Gamepad loss, bridge loss and cable loss all end in Sleep or no output.
- Control traffic is never held up by the display.

**Non-Goals:**
- WiFi, OTA updates, multiple controllers, and haptic or LED feedback on the gamepad.
- Filling the sensor telemetry fields, which belongs to `add-body-sensing`.

## Decisions

### D1. Layout

```
core/protocol/            frame encode/decode, CRC, command/telemetry structs (hardware-free)
core/gamepad_mapping/     pad state -> desired command state, edge/hold detection (hardware-free)
firmware/bridge/          PlatformIO project (ESP-IDF + Arduino component + Bluepad32)
  platformio.ini, sdkconfig.defaults, main/ or src/
  src/main.cpp            setup/loop: BP32.update, mapping, link TX/RX, display
  src/link.*              UART1 on GPIO27 TX / GPIO26 RX
  src/display.*           status screen
firmware/servo2040/src/link.*   UART1 on GPIO20 TX / GPIO21 RX
```

`core/gamepad_mapping` sees only a plain `PadState` struct (axes, button bitmasks, a time in ms). The bridge fills it from Bluepad32's `Controller`, so the mapping can be host-tested without Bluepad32.

### D2. Frame format

```
offset size field
0      1    start marker 0xA5
1      1    protocol version (1)
2      1    type (1 = command, 2 = telemetry)
3      1    sequence number (wraps)
4      1    payload length N
5      N    payload (little-endian, fixed layout per type/version)
5+N    2    CRC-16/CCITT-FALSE over bytes 1..4+N
```

- **Command payload** (12 bytes): mode u8, translate x i8, translate y i8, rotate i8, height u8, speed u8, gait u8, stride u8, flags u8 (bit0 controller connected, bit1 emergency stop), clear counter u8, mode-request counter u8, and 1 reserved byte.
- **Mode-request counter:** the bridge increments it on every mode toggle or combination (Stand/Walk, Sleep, Calibrate). The robot feeds it to `protection::resumeGate` (added in `port-movement-to-servo2040`), so that after a fault the body stays asleep until the operator makes a new request. Clearing the fault alone never restarts motion, even though every frame repeats the desired mode.
- **Telemetry payload** (fixed, about 32 bytes):
  - body mode u8, gait u8, outputs on u8, fault bitmask u16;
  - voltage mV u16, current mA u16;
  - valid frames u16, discarded frames u16;
  - sensor block: availability bitmask u8, ToF mm u16, ToF status u8, pitch and roll in centidegrees i16 ×2, plus reserve.
- **Why fixed binary rather than text or a serialisation library:**
  - it parses deterministically in fixed time on the RP2040;
  - both sides share one C++ definition;
  - the version byte allows the format to evolve.
- **Why a 1-byte length with resynchronisation on the start marker:** it bounds parser state. A start marker byte inside a payload is harmless, because the CRC rejects a false frame.
- **Abandoning a false start:** the decoder keeps the earliest candidate until its CRC can be checked, so a start marker inside a payload never splits a genuine frame. A false start is abandoned when the line goes quiet: each side calls the decoder's `idle()` when no byte has arrived for longer than a frame takes to send (a tick with no received bytes is enough at both rates). The rescan then recovers any complete frame behind it.

### D3. Rates and baud

- **Baud:** 115200 8N1. A command frame of 19 bytes at 50 Hz uses about 8 % of the link; telemetry of about 39 bytes at 10 Hz uses about 3 %.
- **Robot side:** an interrupt-driven RX ring buffer. Each 20 ms tick drains it and keeps the newest valid command. Telemetry TX happens from the main loop every fifth tick, using the UART FIFO with no blocking writes.
- **Bridge side:** a dedicated FreeRTOS task (priority 5, above Arduino's `loop()` at 1, on the same core) sends a command frame every 20 ms with `xTaskDelayUntil`, samples the e-stop button, and decodes telemetry. `loop()` runs Bluepad32, the mapping and the screen, and hands the newest desired state to the task under a spinlock. Revised from "TX in `loop()`": a full-screen redraw takes about 13 ms at 40 MHz SPI, which would push frames past their 20 ms slot.

### D4. Robot integration

- `link` produces `{MovementCommand, controllerConnected, estop, clearCounter}` from the newest valid frame and stamps `issuedAtMs = millis()` on arrival.
- It keeps the last-seen clear counter and turns each new value into one clear request for `protection::step`. Each change of the mode-request counter advances the robot's own `requestSeq` for the resume gate.
- After a link gap of 200 ms or more (power-up, cable pull, bridge restart), the first frame only sets the counter baselines. A rebooted bridge starts its counters again and its e-stop toggle released; without the baseline, that change would clear the robot's faults and count as a new request, and so restart motion with nobody asking.
- This logic is hardware-free, in `core/protocol/link_receiver`, and host-tested; `firmware/servo2040/src/link` only moves bytes.
- **Emergency stop** becomes a protection input. `core/protection` gains an `estop` input and an `EmergencyStop` fault that inhibits output and clears only when the stop is released and a clear is requested. Its tests live with the other protection tests.
- **Controller disconnected:** the bridge already requests Sleep in that case. The robot also treats a frame without a connected controller as a Sleep request, as a second line of defence.
- The serial command hook becomes `-DSPOODER_SERIAL_COMMANDS`, off by default. When it is on, it replaces the link as the command source.

### D5. Gamepad mapping logic (bridge)

- **State held across frames:**
  - the requested base mode (Stand or Walk), the Sleep toggle and the Calibrate toggle;
  - height, speed, gait and stride;
  - the clear counter;
  - per-button previous state, for edge detection;
  - hold timers for the two combinations.
- **Combination priority:** while `miscBack` is held, `miscHome` presses don't toggle Stand/Walk. This avoids a toggle firing at the start of the Calibrate combination.
- **Stick conversion:**
  - axes are −511..512, mapped to −100..100;
  - the radial dead zone uses a rescaled magnitude `(m − dz)/(1 − dz)`, which is continuous at the edge;
  - the Bluepad32 Y axis is negative for up, so it is negated, giving +Y = forward. This is verified at bring-up.
- **Controller-connected flag:** from the Bluepad32 connect and disconnect callbacks. One controller slot; later connections are rejected.

### D6. Bridge build

- **Project base:** `esp-idf-arduino-bluepad32-template` through PlatformIO, with the pioarduino platform (template pin `55.03.31-2`) and ESP-IDF 5.5.x.
- **Pinning:** the template commit and the Bluepad32 release are pinned, through the template's component manifest.
- **Console:** the Bluepad32 USB console stays on UART0. The link uses UART1, routed to GPIO27 and GPIO26 through the GPIO matrix.
- **Display library:** LovyanGFX, pinned to a version, with a panel config for ST7789 135×240 on the pins above. TFT_eSPI's bundled copy isn't compatible with Arduino 3.x.
  - Fallback: ESP-IDF `esp_lcd` with an ST7789 panel driver and a small bitmap font. The first bridge task verifies LovyanGFX builds and runs; if it doesn't, it switches to the fallback.
- **Screen redraws:** run in `loop()` on core 1, at most every 250 ms, drawing only the fields that changed. A full-screen redraw happens only when the link state changes.
- `mise` tasks: `build-bridge`, `upload-bridge` and `monitor-bridge`, which call `pio run -d firmware/bridge`.
- **Pinned versions (verified building on 2026-10-17):**

  | Component | Pin |
  |---|---|
  | Bluepad32 template | `external/bluepad32-template`, tag 5.0.0-beta0, commit `2c881858b6d7` |
  | Bluepad32 | 5.0.0 (vendored in the template) |
  | Arduino core | 3.3.12, template submodule commit `94afccf35fb1` |
  | BTstack | template submodule commit `8a8b616e04b0` |
  | pioarduino platform | 55.03.31-2, which provides ESP-IDF 5.5.1 |
  | pioarduino PlatformIO core | 6.1.18, installed by the platform into `.pio-core/penv` |
  | LovyanGFX | `external/LovyanGFX`, tag 1.2.32, commit `e2cc321544fd` |

  The template's README asks for ESP-IDF 5.5.5; the pinned platform provides 5.5.1 and the build succeeds. Revisit if Bluetooth misbehaves at bring-up.
- **Two PlatformIO cores:** mise's PlatformIO 6.2.0 (host tests) and pioarduino's 6.1.18 pin different SCons versions; building with 6.2.0 reinstalls SCons under the running build and fails. The bridge tasks therefore run the platform's own core, with `PLATFORMIO_CORE_DIR=.pio-core` so its ESP-IDF Python environment is built from mise's Python 3.13. (`pydantic-core` does not yet build on Python 3.14.) `pyyaml` is added to mise's PlatformIO, because the platform imports it while bootstrapping.
- **Namespace clashes on ESP-IDF:** the bridge's link module is `robot_link`, because POSIX `link()` is visible there. `core/movement/vectors.h` calls its helpers `interpolate`, not `lerp`, because C++20 `std::lerp` is visible.

### D7. Testing

- **Host tests (Unity, PlatformIO `native`):**
  - `core/protocol`: round trips, CRC rejection, resynchronisation over garbage, version and type rejection, and sequence wrap;
  - `core/gamepad_mapping`: every row of the mapping table, dead-zone continuity, edge and hold behaviour, combination priority, and the connect/disconnect defaults;
  - `core/protection`: the emergency-stop fault.
- **Hardware checks:** the link at 115200 with a cable-pull test, e-stop latency on the robot, and screen updates under load.

## Risks / Trade-offs

- [The Bluepad32 template and pioarduino versions move quickly; the template was updated on 2026-10-14] → Pin the commit and platform version, and record them.
- [Display library compatibility with Arduino 3.x] → Verify LovyanGFX early. `esp_lcd` is the fallback.
- [GPIO0 is a strapping pin, and holding that button at boot enters download mode] → Use GPIO35 for the emergency stop. GPIO0 stays unused, or becomes a non-critical page switch.
- [Bluetooth Classic latency and coexistence] → Disable WiFi. The 1 s stale timeout and the explicit disconnect handling bound the effect of link hiccups.
- [Ground loops or noise through the servo ground] → Wire ground only to logic GND (J6 pin 16), as documented in the hardware notes.
- [Both boards powered over USB at once while sharing 5 V] → Don't connect the boards' 5 V rails. Share only GND, TX and RX.

## Migration Plan

1. Land `core/protocol` and `core/gamepad_mapping` with their tests, then the robot `link`, then the bridge firmware.
2. Bring up the link with the bridge connected over a bench cable and the robot on the bench supply with no servos, using telemetry on the screen. Then add the gamepad. Then test with servos, with the robot on a stand.
3. **Rollback:** build the robot firmware with `-DSPOODER_SERIAL_COMMANDS` to drive it over USB again.

## Open Questions

- Which exact gamepad will be used. This only affects button labels on the screen and the Y-axis sign check, not the design.
