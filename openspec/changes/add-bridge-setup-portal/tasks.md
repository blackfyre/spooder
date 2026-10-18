# Tasks

## 1. Hardware-free core (host-tested)

- [ ] 1.1 `core/setup/portal_controller` (design D4). Verify host tests for connected within 1 s of a poll and disconnected after, mode buttons advancing the mode-request counter, zero translate and rotate, and Sleep on disconnect.
- [ ] 1.2 `core/setup/calibration_sync` (design D5). Verify host tests for one set per period, oldest pending first, latest value only, confirmation by request id, the resend after 200 ms with the same id, and a clamped or rejected result replacing the shown value.
- [ ] 1.3 `core/setup/calibration_export` (design D7). Verify the golden test against `test/golden/calibration_defaults.cpp`, and an export of a changed table.
- [ ] 1.4 Setup variant of `core/status/status_screen` (design D8). Verify host tests for the content list in the spec, and that every line fits.

## 2. Bridge firmware (TTGO T-Display)

- [ ] 2.1 Setup-mode boot path (design D2): GPIO0 held 1 s during the logo, RTC magic, `esp_restart()`; in `main.c`, skip Bluetooth and start Arduino and the setup task. Verify the build, then on the board that both paths boot (requires the TTGO on USB).
- [ ] 2.2 WiFi soft AP (one station), the NVS-kept random password, `DNSServer` answering everything, and the HTTP endpoints including captive-portal probes (design D6). Verify the build.
- [ ] 2.3 The portal page (design D6) in `portal_page.h`: joint editor with nudges, mode buttons, telemetry, e-stop and link indicators, Export and Exit; plain HTML and JavaScript only. Verify the build and the page size (under 16 KB).
- [ ] 2.4 Wire the portal controller and calibration sync into `robot_link` in setup mode (command frames from the portal, one calibration set per period, reports decoded). Verify the build.
- [ ] 2.5 The setup screen with the QR code (design D8). Verify the build.

## 3. Verification and documentation

- [ ] 3.1 Run `mise run test`, `mise run build` and `mise run build-bridge` from a clean state. Verify all pass with no warnings in our sources.
- [ ] 3.2 Bring up on the hardware (requires the user), recording results here:
  - enter and leave setup mode; a power-up always starts normally;
  - a phone joins via the QR code and the portal opens by itself;
  - Calibrate from the portal; nudge one servo's offset and end points, watching the rate limit; changes outside Calibrate are rejected;
  - the e-stop, and a phone walking away (robot sleeps), during calibration;
  - export, paste into `calibration_defaults.cpp`, rebuild and flash, and confirm the robot reports the same table;
  - restart the bridge into normal operation and walk with the live values.
- [ ] 3.3 Update `CLAUDE.md`, `spooder-firmware-shape` (setup mode) and `spooder-hardware-check` (setup mode and the portal calibration workflow). Verify that every symbol and path mentioned exists (`grep`).
