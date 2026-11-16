# Design

## Context

See proposal.md for the motivation and specs/ for the requirements. The rules come from `.claude/skills/spooder-firmware-shape/SKILL.md` ("Mode-dependent behaviour is an explicit state machine", "No flags that duplicate a state"). Line numbers below are at `631491b`.

- **Boot mode.** `core/wifi_drive/drive_mode::select()` returns `drive_mode::Mode {Bluetooth, Wifi, Setup}`. `firmware/bridge/main/wifi_drive.cpp:39-58` (`bridge_boot_mode_select`) maps it to the C enum `bridge_boot_mode {BRIDGE_BOOT_BLUETOOTH, BRIDGE_BOOT_WIFI, BRIDGE_BOOT_SETUP}` with a `default:` that returns Bluetooth. `main.c:36-42` runs `setup_mode_start()` for setup, `wifi_drive_start()` for anything else that is not Bluetooth, and falls through into Bluepad32 for Bluetooth. `main.c` is C and compiled in the bridge's `main` component with `-Wall -Wextra -Werror` (`build-warnings`), so `-Wswitch` is fatal there as in the C++ sources.
- **Link mode.** `link.cpp` has `portalMode` and `handheldMode`, set before the task starts by `start()`, `startPortal()` and `startHandheld()` (called from `bridge.cpp:93`, `setup_mode.cpp:350` and `wifi_drive.cpp:237`), and read in `sendCommand`, `takeFrames` (twice), `receive` (twice) and `linkTask`.
- **Emergency stop.** `sendCommand` computes `stop = emergencyStop || (handheldMode && handheld_controller::emergencyStop(handheld))` under the lock. `status()` returns the toggle alone; `handheldView()` returns the handheld's flag alone, each under its own lock section; `wifi_drive.cpp:207` ORs them for the screen. `bridge.cpp` (rumble) and `setup_mode.cpp` (the portal's `estop`) read `status().emergencyStop` in modes without a handheld, where the toggle is the whole stop. `HandheldView::emergencyStop` has no other reader. `tools/virtual-robot/src/wifi_bridge.cpp:171` computes `gpioStop || handheld_controller::emergencyStop(controller_)` for its own command frame.
- **Idle gap.** `receive()` keeps `static uint32_t lastByteMs` and `static bool idleHandled = true`: bytes arriving set `idleHandled = false`; once `RX_IDLE_MS` (10 ms) passes without a byte, the decoder's `idle()` runs once and `idleHandled` is set again.

## Goals / Non-Goals

**Goals:**

- One representation of the bridge's boot mode from the decision to every branch on it, with every branch exhaustive and checked by the compiler.
- One computation of the emergency stop the command frames carry, shared by the frames, the screen and the virtual robot.
- No function statics standing for a state in the link's receive path.

**Non-Goals:**

- The other seven items of the brief (see Follow-ups).
- Moving the link task itself into `core/`; it is UART, FreeRTOS and lock glue.
- Any change to the link protocol, the timings or the screen layout.

## Decisions

### D1. `drive_mode::Mode` is the one boot-mode type

`robot_link::start(drive_mode::Mode mode)` replaces the three start functions. The link stores it in one file-scope `static drive_mode::Mode mode`, written before the task is created and never again (as the bools were), so it needs no lock. Each branch that today tests `portalMode` or `handheldMode` becomes a `switch (mode)` naming all three enumerators and no `default:`; a small helper per question (`static bool relays()`, `static bool sendsCalibration()`) is acceptable where a `switch` inside an expression would be clumsy, as long as the helper is itself the exhaustive `switch`. The callers pass `drive_mode::Mode::Bluetooth`, `::Setup` and `::Wifi`.

A new enum for the link was considered and rejected: the drive mode already is the link's mode, one-to-one, and a second enum would be the duplicate the rule forbids.

### D2. The C enum is a pinned mirror

`main.c` cannot include the C++ `drive_mode.h`, so `bridge_boot_mode` stays in `wifi_drive.h`, documented as the C mirror of `drive_mode::Mode`. `wifi_drive.cpp` converts with one `constexpr enum bridge_boot_mode toBootMode(drive_mode::Mode m)`, an exhaustive `switch` over the three enumerators without `default:`, each case returning its C value. Below it, for each mode, `static_assert(toBootMode(drive_mode::Mode::Bluetooth) == BRIDGE_BOOT_BLUETOOTH && (int)BRIDGE_BOOT_BLUETOOTH == (int)drive_mode::Mode::Bluetooth, ...)` and so on for all three. `bridge_boot_mode_select` returns `toBootMode(drive_mode::select(...))`. A plain cast was considered and rejected: a fourth `drive_mode::Mode` handled in the link but not mirrored would cast to an unnamed C value that `app_main`'s `switch` silently skips into Bluepad32, with no warning (review round 1). `app_main` dispatches with `switch (boot)` over the three values and no `default:`, the Bluetooth case jumping on to the Bluepad32 start-up below as today and an unmatched value aborting. A fourth `drive_mode::Mode` then fails the build in `toBootMode` and in `link.cpp`'s `switch` statements (`-Wswitch`), each independently; a fourth C value fails it in `app_main`; a reordered, renumbered or wrongly mapped value fails a `static_assert`.

The boot decision itself (`drive_mode::select`) is already host-tested (`test/test_drive_mode`), and the conversion is checked at compile time; only `app_main`'s three-case dispatch to `setup_mode_start()`, `wifi_drive_start()` and Bluepad32 is left to the hardware check (task 4.1). It is C calling hardware start-up functions, and factoring it into a host-testable table would add an indirection larger than the three cases it checks.

### D3. The emergency stop is one rule, computed in one place

`core/wifi_drive/handheld_controller` gains `bool commandStop(drive_mode::Mode mode, const Controller &c, bool bridgeStop)`, an exhaustive `switch` over the mode without `default:`: in Bluetooth drive and setup mode `bridgeStop` (the bridge's own toggle) alone; in WiFi drive `bridgeStop` or the handheld's flag, which stays set through a lapse until an accepted frame clears it, as the `wifi-drive` requirement of `add-wifi-drive` already says. It is pure, and host tests cover all three modes, so the choice of rule per mode is checked deterministically and not only the OR (review round 1).

In `link.cpp` one function, `static bool commandedStop()`, called with the lock held, returns `handheld_controller::commandStop(mode, handheld, emergencyStop)`. `sendCommand` and `status()` both call it, and nothing else in the link computes a stop, so the frame and the status read the same rule; `status()` fills `Status::emergencyStop` from it in the same lock section as the telemetry. Because `handheld_controller::emergencyStop()` takes no time, the stop reported by `status()` is the one the next frame carries unless a datagram or a button press arrives in between, which is the same freshness every other `Status` field has. `HandheldView::emergencyStop` is removed (no reader is left) and `wifi_drive.cpp` passes `robot_link::status()` to the screen unchanged.

The virtual robot's `WifiBridge::beforeTick` calls `handheld_controller::commandStop(drive_mode::Mode::Wifi, controller_, gpioStop)`.

The frame encoding of the stop (`gamepad_mapping::toCommand`, `protocol::encodeCommand`) is already host-tested; what stays untested on the host is the two calls of `commandedStop()` in the FreeRTOS and UART glue, which task 2.2 checks with a grep and task 4.1 on the boards.

Storing the stop the last frame carried was considered and rejected: it would lag the toggle by up to a period, and recompute nothing that the rule does not already give.

### D4. The idle gap is an explicit two-state machine in `core/protocol`

The function statics become a small pure state machine next to the decoder it serves, in `core/protocol/protocol.{h,cpp}` (no new source file, so no source-list change):

- `protocol::RxGap`, a struct holding `enum class State : uint8_t { Quiet, Receiving }`, starting `Quiet` (today's `idleHandled = true`), and `uint32_t lastByteMs`, meaningful in `Receiving` only;
- one transition function, `bool update(RxGap &g, bool bytesArrived, uint32_t nowMs, uint32_t idleMs)`: bytes arriving enter `Receiving` and stamp `lastByteMs`; in `Receiving` without bytes, `nowMs - lastByteMs >= idleMs` (unsigned, so correct across the `millis()` wrap) returns to `Quiet` and returns true, once, which is when `receive()` calls `decoder.idle()`.

`link.cpp` holds one file-scope `protocol::RxGap`, owned by the link task, and passes `RX_IDLE_MS` (10 ms), which stays the bridge's constant. Placing it in `core/` was first rejected as more than the change needs, then adopted after review round 1: the bridge's receive path has no host test, and a wrong gap (firing early, repeatedly or never) would discard or strand partial frames. In `core/` it is host-tested in `test/test_protocol`. The robot keeps its per-tick idle in `robot_tick` (a whole 20 ms tick without a byte), which this change does not touch.

## Assumptions

- The brief's "behaviour unchanged except where stated" admits the one visible difference: the WiFi drive screen now reads the emergency stop from one snapshot instead of ORing two.
- "Keep any C enum only as a static_assert-pinned mirror with no default" applies to `app_main`'s dispatch as well as to `bridge_boot_mode_select`, since `main.c`'s `else` is the same silent fallback in the other direction.
- `core/` changes are limited to the one `handheld_controller` function the shared rule needs, the `protocol::RxGap` state machine (D4) and a stale comment in `core/status/status_screen.h` (`Inputs::emergencyStop`, already the combined stop in WiFi drive); this does not overlap `explicit-state-core`, which converts `core/` state.
- `add-wifi-drive` is not archived, so its `wifi-drive` capability is not in `openspec/specs/`; this change states the e-stop rule against `status-display` (the screen) and leaves `wifi-drive` alone. Its existing e-stop requirement is unchanged and is what `commandStop` implements.
- Reflashing only the bridge is enough: no frame changes, so `protocol::VERSION` stays as it is (3 when this change was written, 4 since `body-stopped-telemetry`, onto which it was rebased).
- The brief's line numbers were checked at `631491b` and match.
- The code after an exhaustive `switch` on the drive mode, which the compiler needs and only an out-of-range value (memory corruption) can reach, calls `abort()` in the bridge (`toBootMode`, `relays()`, `syncsCalibration()`, and `app_main`, where only the Bluetooth case jumps on to BTstack and an unmatched value aborts; security review) instead of returning another mode's answer, so no fallback maps an unknown mode to another one (review round 1); the bridge restarts and decides its boot mode again. The core's `commandStop` keeps `return true`: stopping is a fail-safe answer, not another mode's rule.
- D1's helpers are `relays()` (WiFi drive) and `syncsCalibration()` (setup mode: it both sends calibration sets and follows the reports, so it is named for both); `portalRequestMode`'s parameter is renamed `requested` so it does not shadow the file-scope `mode`.

## Risks / Trade-offs

- [The bridge's link task is on the safety path (the e-stop and the heartbeat)] → the e-stop rule, including its choice per mode, and the receive idle gap move into host-tested core functions; the link change is mechanical and every branch is compiler-checked; the hardware check (task group 4) exercises the GPIO35 toggle and the handheld's flag on the boards before the change is called done.
- [Exhaustive `switch` without `default:` relies on `-Wswitch` being fatal] → `build-warnings` makes it so for `main/`; task 2.1 proves it with deliberate missing cases, including a fourth mode handled in the link but not mirrored in C.

## Follow-ups

The brief's remaining items, each independent of this change and of one another, for later changes:

1. **Bridge display phase** (`display.cpp:77-93, 207-226, 317-321, 385-388, 407-443`): `phase`, `logoHeld`, `firstDraw`, `lastDrawMs` and `shown[]`, with `setupStart()` setting `phase = Status` to stand for the setup screen. Target: `Phase {Logo, BootLog, Status, Setup}` with one `advance()`; `update()` refuses to draw in `Setup`.
2. **Robot servo output** (`firmware/servo2040/src/servo_output.cpp:31-45, 61-66, 93-188`): `on`, `inhibited`, `pinsForcedLow`, `forcedOff`, `switchedOffAt`, `legOn[]`, and "never set" encoded both as `desiredUs == 0` and `legSet[]`. Target: `enum Output {Released, Settling, On, ForcedOff}` with one transition function; `ForcedOff` terminal until restart and refusing everything; the inhibit and force-off semantics kept exactly; a hardware check task. Safety-critical, so its own change.
3. **Setup mode flags** (`setup_mode.cpp:65-66, 280-281, 306-324`): `exitRequested` with `exitAtMs`, and `screenDrawn` with `lastScreenMs`.
4. **Bluetooth controller pointer** (`bridge.cpp:36, 49-67, 141`): `controller == nullptr` used as state alongside `pad.connected`.
5. **Virtual robot bridge source** (`tools/virtual-robot/src/main.cpp:91-101, 174-192, 256, 274-281, 405-421`): implied by `wifi.isOpen()` and `bridge.isOpen()`. Target: `enum BridgeSource {Simulated, Pty, Wifi}`.
6. **Virtual robot request sentinel** (`tools/virtual-robot/src/sim.cpp:9-24`): `framesUntilRequest = -1` survives leaving the Gamepad state, so `bridge gamepad`, `step 1`, `bridge idle`, `step 5` increments `modeCounter` while Idle. A behaviour bug: cancel it in `setState` or make it part of the Gamepad state, with a test. The smallest of the follow-ups and a candidate to go first.
7. **Virtual robot WiFi bridge session shadow** (`tools/virtual-robot/src/wifi_bridge.cpp:107-143`): `live_`, `seenEpoch_` and `acceptedAtStart_` shadowing `handheld_controller`'s session, and the `lastTickStartNs_ == 0` sentinel. (Its e-stop line is handled here, D3.)
