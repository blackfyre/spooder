---
name: spooder-firmware-shape
description: Structural rules for the Spooder firmware on the Pimoroni Servo 2040 — core/ versus firmware/ split, the core 0 control loop, the power-protection layer between commands, movement and servo output, J6 pin allocation, PIO/DMA usage, state machines, named constants and failure handling. Use before adding a module or sensor, moving code, using the second core, or reviewing a change for structure.
---

# Spooder firmware shape

Adapted from architecture guidance on modules, explicit state machines, named constants and test boundaries, applied to a single-binary microcontroller firmware. Intended behaviour lives in OpenSpec (`openspec/specs/`); this skill covers structure.

## Two layers

| Layer | Where | Rules |
|---|---|---|
| Hardware-free core | `core/` (`movement/`, `protection/`) | Only `<math.h>`, `<stdint.h>` and other `core/` headers. No pico-sdk, no pimoroni, no globals with hidden state; pass state structs explicitly. Every behaviour change needs host tests (`test/`, `mise run test`). Built into the firmware as `spooder_core` with `-Wall -Wextra`. |
| Hardware adapters | `firmware/servo2040/src/` | Thin: read hardware into core inputs, or apply core outputs to hardware. No decisions that could live in `core/`. Each adapter exposes `setup()` plus a small API in its header; file-local state is `static`. |

New logic goes into `core/` unless it truly needs hardware. A controller mapping, a filter or a protocol parser is core logic.

## The control loop (core 0)

`main.cpp` runs a 20 ms period loop (`LOOP_PERIOD_MS`, `sleep_until`; an overrun restarts the period):

```
reading = power_monitor::read()
prot    = protection::step(state, reading, now, clearFromPreviousTick)  // first, always
servo_output::setInhibited(!prot.allowOutput)
input   = serial_commands::poll(now)          // bounded per tick; later: the bridge link
run     = protection::supervise(supervisor, prot, input.mode, input.requestSeq)
if run.stopBody:  servo_output::stop()                 // output-cutting fault: body stops
if run.startBody: body::init(bodyState)                // boot, and restart after such a fault
if run.runBody:   body::step(bodyState, cmd with run.mode, now); servo_output::apply(bodyState)
status_leds::show(...)
```

- **Protection runs first in the tick.** Nothing in the tick can delay a trip, and input reading is bounded (`MAX_CHARS_PER_POLL`).
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
| USB | stdio console |
| UART | **none**: GPIO0/1 are servos; never enable stdio UART |

J6 header (unpopulated; solder a header) allocation:

| J6 pin | GPIO | Use |
|---|---|---|
| 14 / 13 | 20 / 21 | UART1 TX/RX to the gamepad bridge (`add-gamepad-bridge`); enable the RX pull-up |
| 10 / 9 | 26 / 27 | I²C1 SDA/SCL to the sensors (`add-body-sensing`); needs external pull-ups |
| 8 | 28 | IMU interrupt |
| 12 | 19 | spare (`I2C_INT`) |
| 15 / 16 / 17 | — | 3V3 / GND / 5V |

The board has no pull-ups on GPIO19–21 or 26–28. Check for clashes before claiming any pin or PIO resource.

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

- **Gates:** `mise run test`, plus the firmware build with limits configured, with no new warnings in `core/` or `firmware/`.
- **Logic tests** live with `core/`. Each bug fix there gets a regression test that fails without the fix.
- **Hardware behaviour** is checked with the `spooder-hardware-check` skill.
