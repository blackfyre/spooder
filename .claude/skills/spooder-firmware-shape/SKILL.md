---
name: spooder-firmware-shape
description: Structural rules for the Spooder firmwares (robot on the Pimoroni Servo 2040, gamepad bridge on the TTGO T-Display) — core/ versus firmware/ split, the core 0 control loop, the power-protection layer between commands, movement and servo output, the bridge link boundary, J6 pin allocation, PIO/DMA/UART usage, the bridge's task layout, state machines, named constants and failure handling. Use before adding a module or sensor, moving code, using the second core, changing the link, or reviewing a change for structure.
---

# Spooder firmware shape

Adapted from architecture guidance on modules, explicit state machines, named constants and test boundaries, applied to a single-binary microcontroller firmware. Intended behaviour lives in OpenSpec (`openspec/specs/`); this skill covers structure.

## Two layers

| Layer | Where | Rules |
|---|---|---|
| Hardware-free core | `core/` (`movement/`, `protection/`, `protocol/`, `gamepad_mapping/`, `status/`) | Only `<math.h>`, `<stdint.h>` and other `core/` headers (`status/status_screen` also uses `<stdio.h>` for `snprintf`). No SDKs, no globals with hidden state; pass state structs explicitly. Every behaviour change needs host tests (`test/`, `mise run test`). Built into the robot as `spooder_core` with `-Wall -Wextra`; the bridge compiles the parts it uses into its `main` component. |
| Hardware adapters | `firmware/servo2040/src/`, `firmware/bridge/main/` | Thin: read hardware into core inputs, or apply core outputs to hardware. No decisions that could live in `core/`. Each adapter exposes `setup()`/`start()` plus a small API in its header; file-local state is `static`. |

New logic goes into `core/` unless it truly needs hardware. A controller mapping, a filter or a protocol parser is core logic.

## The control loop (core 0)

`main.cpp` runs a 20 ms period loop (`LOOP_PERIOD_MS`, `sleep_until`; an overrun restarts the period):

```
input   = readInput(now)                      // link::poll + usb_link::poll, then link_receiver::arbitrate;
                                              // or serial_commands::poll. Bounded, never blocks
reading = power_monitor::read()
prot    = protection::step(state, reading, now, input.clearFaults, input.emergencyStop)
servo_output::setInhibited(!prot.allowOutput)
requested = command fresh (≤ 1 s) ? input.mode : Sleep
run     = protection::supervise(supervisor, prot, requested, input.requestSeq)
if run.stopBody:  servo_output::stop()                 // output-cutting fault: body stops
if run.startBody: body::init(bodyState)                // boot, and restart after such a fault
link::processCalibration / usb_link::processCalibration(requested table, calibrating, prot.allowOutput)
joint_calibration::step(applied, requested, dt)        // 60°/s towards the requested table
if run.runBody:   body::setOffsets(applied offsets); body::step(bodyState, cmd with run.mode, now);
                  servo_output::apply(bodyState, applied)
status_leds::show(...)
every fifth tick: telemetry on both links      // non-blocking, whole frames or nothing
reports due (after a set, 1 Hz in Calibrate, ≤ 5 Hz): calibration report on each link
```

- **Input, then protection, at the top of the tick.** Input comes first so an emergency stop in a frame that has already arrived acts in this tick (within 20 ms). Both sources are bounded and non-blocking (`MAX_CHARS_PER_POLL`; the link drains at most its 256-byte ring), so they cannot meaningfully delay a trip.
- **Input sources count explicit mode requests (`requestSeq`).** After any fault, nothing moves until the count changes, so clearing a fault never restarts motion by itself.
- **Recovery:**
  - Faults that cut output stop the body; it restarts through Initialise once output is allowed and a new request has been made.
  - Sleep-only faults wake through Stand.
  - `core/protection/supervisor` owns this; don't add parallel start/stop logic elsewhere.

- **Protection rewrites requests and gates output, but never changes body mode itself.** Movement owns all transitions (`locomotion` spec).
- **The body starts only when output is allowed,** so Initialise runs with the servos live.
- **Nothing in the loop may block.** Debug output is USB stdio with `PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000`, so a host that isn't reading drops output instead of stalling the loop. Don't add `sleep_ms` or blocking I/O.
- **One core today.** The `add-body-sensing` change introduces core 1 for sensor I/O. Data then crosses cores only through a single-writer seqlock snapshot that is never shared mutably. Construct `ServoCluster` and everything else that allocates on core 0, *before* launching core 1 (pimoroni's `pwm_alloc` warning).

## Hardware resources

| Resource | User |
|---|---|
| PIO 0 SM 0 + 1 DMA channel + `DMA_IRQ_0` (shared handler) | `ServoCluster` (`servo_output`) |
| PIO 1 SM 0 + DMA + a repeating timer | WS2812 LEDs (`status_leds`, 10 fps) |
| ADC via the 74LV4051 mux (GPIO22/24/25 address, GPIO29 `SHARED_ADC`) | `power_monitor` (current at address 7, voltage at 6); sensor headers at 0–5 |
| USB | stdio console, plus link frames (`usb_link`) in default builds; `CFG_TUD_CDC_TX_BUFSIZE=256` so a whole report fits |
| UART1 on GPIO20/21 + `UART1_IRQ` | the bridge link (`link`): RX ring and a 256-byte whole-frame TX queue under the interrupt |
| UART0 / stdio UART | **none**: GPIO0/1 are servos; never enable stdio UART |

J6 header (unpopulated; solder a header) allocation:

| J6 pin | GPIO | Use |
|---|---|---|
| 14 / 13 | 20 / 21 | UART1 TX/RX to the gamepad bridge (`link`); RX pull-up enabled in `link::setup` |
| 10 / 9 | 26 / 27 | I²C1 SDA/SCL to the sensors (`add-body-sensing`); needs external pull-ups |
| 8 | 28 | IMU interrupt |
| 12 | 19 | spare (`I2C_INT`) |
| 15 / 16 / 17 | — | 3V3 / GND / 5V |

The board has no pull-ups on GPIO19–21 or 26–28. Check for clashes before claiming any pin or PIO resource.

## The links

- **Boundary:** `core/protocol` owns framing (versioned, CRC-16/CCITT-FALSE, fixed little-endian payloads, incremental decoder). `core/protocol/link_receiver` owns what a decoded frame means to the robot. The adapters (`link` on both boards, `usb_link` on the robot) only move bytes. Both firmwares compile the same protocol code; change it in one place, bump `protocol::VERSION` when the layout changes, regenerate `test/golden/protocol_v2.txt` (`SPOODER_WRITE_GOLDEN=1 mise run test`) and flash both boards together.
- **Frames (version 2):** command (front-end to robot, 20 ms), telemetry (robot, 100 ms; byte 21 holds the source flags), calibration set (front-end, one joint) and calibration report (robot, the whole table, 135 bytes).
- **Two links on the robot:** the bridge on UART1 and a computer on USB, each with its own `Decoder` and `link_receiver::Receiver`. `link_receiver::arbitrate` picks the one reporting a fresh, connected controller; both at once is a conflict (Sleep, `TELEMETRY_SOURCE_CONFLICT`). The e-stop is either link's; a fault clear and `requestSeq` come only from the source in control.
- **USB shares the port with `printf`.** Frames and text are both written from the core 0 loop, so neither lands inside the other. `usb_link` writes a frame only when the CDC buffer has room for all of it. Keep debug text printable ASCII.
- **Calibration:** sets are queued per link and processed after protection, only in Calibrate with output allowed (`processCalibration`, clamps in `joint_calibration::clampJoint`); a repeated request id is a retransmission. Reports carry the requested table; `ReportSchedule` sends one after each set and at 1 Hz in Calibrate, at most 5 Hz per link.
- **Frames carry desired state, never events.** Counters stand in for events: the fault-clear counter (one clear per change) and the mode-request counter (advances `requestSeq` for the resume gate). After a link gap of 200 ms or more, the first frame only sets the counter baselines, so a restarted bridge cannot clear faults or count as a request.
- **Staleness:** commands are stamped on arrival; the body's 1 s stale rule turns a silent link into Sleep, and a stale command never starts a stopped body.
- **Decoder false starts:** the earliest candidate is kept until its CRC is checked. Each side calls `Decoder::idle()` when the line has been quiet longer than one frame, so a false start cannot hide a frame behind it.
- **Rates:** commands every 20 ms (bridge), telemetry every 100 ms (robot), 115200 8N1 on UART1. A report takes about 11.7 ms on UART1.

## The bridge (TTGO T-Display, ESP32)

| Context | Core | Work |
|---|---|---|
| BTstack / Bluepad32 | 0 | Bluetooth; callbacks into `bridge.cpp` |
| `robot_link` task, priority 5 | 1 | every 20 ms (`xTaskDelayUntil`): sample the GPIO35 e-stop button, send a command frame, decode telemetry and calibration reports |
| Arduino `loop()`, priority 1 | 1 | `BP32.update()`, `gamepad_mapping::update`, hand the desired state to the link task, redraw the screen (≤ 4 Hz, changed lines only) |

- Shared state between `loop()` and the link task is copied under one spinlock (`portMUX`); nothing else is shared.
- The screen must never delay command frames: keep drawing in `loop()`, never in the link task.
- One controller slot (`CONFIG_BLUEPAD32_MAX_DEVICES=1`, plus a refusal in `onConnected`). WiFi is never started.
- Pins: link UART1 TX GPIO27, RX GPIO26; e-stop GPIO35 (input only, board pull-up); display SPI MOSI 19, SCLK 18, CS 5, DC 16, RST 23, backlight 4. GPIO0 is a strapping pin: leave it unused.
- The Bluepad32 console owns UART0 (the USB bridge); use `Console`, not `Serial`.

## State and constants

- **Mode-dependent behaviour is an explicit state machine:** one `enum class`, one transition function with its guards. Examples are `body::Mode` with `resolveMode()`, and protection's latched-fault bitmask.
- **No flags that duplicate a state.**
- **Hardware constants are named:** pins come from `servo2040::*`; limits come from `config.h`/`power_limits.h`. Tunable rates are `constexpr` in the module that owns them.
- **Safety limits are configuration, not code:**
  - `power_limits.h` is required at build time;
  - `config.h` checks order and range with `static_assert`;
  - never add a default that would let the firmware build without deliberate limits.

## Failures

- **Hardware set-up failure** (for example, `servo_output::setup()` can't claim PIO or DMA): report, show it on the LEDs, and never move.
- **Out-of-range supply:** handled only by `core/protection`, and never bypassed. Code that could switch servo output on must go through `servo_output`, which honours the inhibit.
- **Invalid IK targets** are rejected before they're recorded, and reported once (`newlyRejected`).
- **Reporting:** debug output on USB stdio, plus the status LEDs. No `printf` in hot paths beyond the once-per-second report and once-per-event messages.

## Verification

- **Gates:** `mise run test`; `mise run build` with limits configured, with and without `--serial`; `mise run build-bridge`. No new warnings in `core/` or `firmware/`.
- **Logic tests** live with `core/`. Each bug fix there gets a regression test that fails without the fix.
- **Hardware behaviour** is checked with the `spooder-hardware-check` skill.
