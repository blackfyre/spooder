# Design

## Context

- **Depends on `add-gamepad-bridge`** (see its design):
  - the link protocol (`core/protocol`, version 1), `link_receiver` on the robot, and the `robot_link` task on the bridge;
  - the bridge boot sequence (logo, then start-up log), and the GPIO35 e-stop button.
- **Bridge start-up today:** `firmware/bridge/main/main.c` (copied from the Bluepad32 template) initialises BTstack and Bluepad32 in `app_main` and runs the BTstack loop, which never returns. Bluepad32's Arduino platform then starts Arduino `setup()`/`loop()`.
- **Depends on `add-live-joint-calibration`:** the robot's calibration table, guards and reports; protocol version 2 with calibration set and report frames; the compiled-defaults format with its golden file in `test/golden/`.
- **The ESP32 radio** is shared between Bluetooth and WiFi (see proposal: Why).

## Goals / Non-Goals

**Goals:**
- The radios never run together: setup mode is a separate boot path that never starts Bluetooth.
- The bridge-side logic (portal controller, reconciliation, export) is hardware-free and host-tested.
- The portal is a thin front-end: every guard is on the robot, and the page shows only what the robot reports.

**Non-Goals:**
- Any WiFi use in normal operation.
- Moving the robot from the portal other than the Stand, Calibrate and Sleep mode requests.
- UI code shared with the desktop calibrator.

## Decisions

### D1. Layout

```
core/setup/portal_controller.{h,cpp}      portal polls and buttons -> desired command state
core/setup/calibration_sync.{h,cpp}       pending changes, resend, confirm by report
core/setup/calibration_export.{h,cpp}     reported table -> calibration_defaults.cpp text
firmware/bridge/main/setup_mode.{h,cpp}   WiFi AP, DNS, HTTP API, setup task
firmware/bridge/main/portal_page.h        the portal page (HTML + JS) as a string
```

### D2. Entering setup mode

- During the logo phase, `display`/`bridge` watch GPIO0 (left button). Held for 1 s, the bridge writes a magic value into an `RTC_NOINIT_ATTR` variable and calls `esp_restart()`.
- `main.c` checks at the top of `app_main`: magic set **and** reset reason `ESP_RST_SW`. It then clears the magic, skips BTstack and Bluepad32 entirely, calls `initArduino()`, and starts the setup task. Otherwise it continues the normal Bluetooth start.
- **Why RTC memory, not NVS:** it never wears flash, and power loss clears it, so a normal power-up always starts in normal operation.
- **Why after the logo, not at power-up:** GPIO0 is a strapping pin, and holding it at reset enters the ROM download mode.
- **Leaving:** the portal's Exit calls `esp_restart()` with the magic cleared, or the operator presses RST.
- **Rejected alternative:** starting WiFi alongside Bluetooth and pausing Bluetooth. This leaves both stacks' memory in use and relies on coexistence behaving.

### D3. Robot side

The calibration table, guards, rate limit, reports, protocol version 2 and source arbitration are specified and designed in `add-live-joint-calibration` (its design D2–D6). The portal uses the bridge link; on the robot it is simply the bridge source.

### D4. Portal as the controller (`core/setup/portal_controller`)

- Inputs: page polls (time of the last one) and mode button presses. Output: a `gamepad_mapping::Desired` with zero translate and rotate, so the existing `toCommand` and link path are reused unchanged.
- **Connected** means a poll within the last 1000 ms. Each button press advances the mode-request counter. A disconnect requests Sleep, like a gamepad disconnect.
- The robot's existing rules then apply unchanged: resume gate, stale command, e-stop.

### D5. Reconciliation on the bridge (`core/setup/calibration_sync`)

- The portal's joint edits go into a pending entry per joint, holding the latest value only.
- The link task sends at most one calibration set per 20 ms period, after the command frame. It sends the oldest pending joint, with a new request id.
- The entry is **confirmed** when a report carries that request id. The portal then shows the reported values, which may be clamped or rejected, and the entry is cleared.
- If there is no confirming report within 200 ms, the same frame is resent with the same request id, so the robot treats it as a retransmission.

### D6. Portal web server

- **Arduino `WiFi` (soft AP), `DNSServer` and `WebServer`** from the pinned core. AP limited to one station. The SSID is `spooder-` plus the last four hex digits of the MAC.
- **Password:** 10 random characters from `esp_random()` (no ambiguous characters), stored in NVS namespace `spooder` on first use.
- **Endpoints:**
  - `GET /` serves the page.
  - `GET /api/state` (poll every 250 ms; also the portal heartbeat) returns telemetry, link state, e-stop, the reported table and pending edits.
  - `POST /api/mode`, `POST /api/joint`, `GET /api/export`, `POST /api/exit`.
  - Captive-portal probes (`/generate_204`, `/hotspot-detect.html`, `/connecttest.txt`, `/ncsi.txt`) and any unknown path redirect to `/`.
- **The page:** a single file, about 10 KB, with no external resources (the phone has no internet).
  - Per joint: offset, both end points and direction, with nudge buttons (±0.5°, ±1°, ±5°; ±10 µs, ±50 µs).
  - Mode buttons (Stand, Calibrate, Sleep), live telemetry, a prominent e-stop and link indicator, Export (text area plus copy) and Exit.
- **Setup task:** runs the DNS and HTTP servers on core 1. `robot_link` keeps its own higher-priority task, as in normal operation.

### D7. Export

`calibration_export` writes the complete `calibration_defaults.cpp` in the format fixed by `add-live-joint-calibration` (its D6). Its golden test exports the format fixture table and compares it with `test/golden/calibration_defaults.cpp` byte for byte; the export of the reported compiled defaults must equal the committed defaults file (amended with `add-live-joint-calibration` D6). The page shows the text with a Copy button; the phone has no checkout to write into.

### D8. Setup screen

- Lines: `SETUP MODE`, network name, password, address `192.168.4.1`, portal client yes/no, link state, e-stop.
- A QR code (`WIFI:T:WPA;S:<ssid>;P:<password>;;`, drawn with LovyanGFX `qrcode`) on the right half.
- `status_screen` gains a setup variant, host-tested like the normal one.

### D9. Clarifications made during implementation

- **Two more core helpers** (`core/setup/`), so the entry and credential rules are host-tested too:
  - `setup_entry`: the 1 s hold detector and the boot decision (magic **and** a software restart).
  - `access_point`: the SSID from the MAC, the password generator and check, and the QR text.
- **Calibration sync** (D5):
  - One set is in flight at a time. A report carries only the id of the last set the robot processed, so a second set in flight would hide the first one's confirmation.
  - Ids continue after the robot's last reported id and never equal it, since the robot would take an equal id as a retransmission and change nothing.
  - Edits are relative to the wanted value (the pending one, else the reported one), so quick repeated nudges add up. They need a report first; the robot reports only in Calibrate or after a set, so the portal asks for Calibrate before it offers the editor.
  - A joint edited again while its set is in flight stays pending after the confirmation, and its newest value goes next.
- **Clear faults on the portal:** an e-stop or a trip latches a fault that only a clear removes, and the robot loses its live calibration on a restart. The portal therefore has a Clear faults control, which advances the fault-clear counter like the gamepad's clear combination. It is not a mode request.
- **Mode after a disconnect:** the portal's requested mode stays Sleep until the next mode press, so a page that comes back never resumes motion by itself.
- **Setup screen layout:** text on the left (lines fit 100 px at the screen font), the QR code 135 px square on the right, so it keeps a quiet zone of about two modules. Line 7 shows the body mode, red while a fault is latched.
- **Entering** (D2): the logo stays up while the left button is down, so a hold that starts late in the 1.5 s logo can still complete.
- **Boot path:** setup mode releases the Bluetooth controller's memory (`esp_bt_controller_mem_release`) before anything else, which also makes starting Bluetooth impossible until the next restart. The network objects (`WebServer`, `DNSServer`) are created only in setup mode.
- **Password** (D6): generated with `bootloader_random_enable()` around `esp_random()`, because WiFi is not yet running and the RNG is only truly random with an entropy source on. NVS key `ap_pass` in namespace `spooder`; a stored value that is not a valid password is replaced. `WiFi.persistent(false)` keeps the WiFi configuration out of flash.
- **Heartbeat** (D4): only `GET /api/state` and the portal's button posts count as polls; captive-portal probes do not, so a phone that has joined but closed the page does not keep the robot awake. The link task evaluates the portal controller itself every period, so a stalled web server cannot keep the controller connected. The page stops polling while hidden.
- **HTTP API:** `POST /api/mode` with `m` = `stand`, `calibrate`, `sleep` or `clear`; `POST /api/joint` with `j` (0–17), `f` (`offset` in centidegrees, `p0`, `p180` in µs, `rev`), `v` and `rel=1` for a nudge. `GET /` under another host name redirects to `http://192.168.4.1/`, so the page's requests go to the portal's address.
- **Task layout:** the lwIP task is pinned to core 0 (`CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0`), next to WiFi and the asynchronous DNS server, leaving core 1 to the link task (priority 5) and the setup task (priority 1).
- **IRAM:** with WiFi and the Bluetooth controller both linked, IRAM overflowed. WiFi's IRAM placement is turned off (`CONFIG_ESP_WIFI_IRAM_OPT=n`, `CONFIG_ESP_WIFI_RX_IRAM_OPT=n`); the portal needs no WiFi throughput. A checkout with an existing `firmware/bridge/sdkconfig.ttgo-t-display` must delete it once so these defaults apply.

## Risks / Trade-offs

- [A phone left on the portal keeps the bridge "connected"] → Only while the page polls; closing or locking the phone stops polling, and the robot sleeps within 1 s.
- [Arduino started without Bluepad32's platform glue] → Setup mode uses only `initArduino()` and FreeRTOS tasks, never `BP32`; verify the boot path on the board first (task 3.1).

## Migration Plan

1. After `add-live-joint-calibration` is in place: the core helpers with their tests.
2. Bridge: setup-mode boot path first, then WiFi and the portal, then calibration.
3. Bench test with one servo, then a leg, on a stand.
4. **Rollback:** the bridge from the previous commit; normal operation never starts WiFi.
