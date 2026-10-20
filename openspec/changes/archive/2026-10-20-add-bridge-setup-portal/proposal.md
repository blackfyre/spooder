# Proposal

## Why

In the field there is no computer at hand, but calibration and checks still need a screen bigger than the bridge's. A setup portal on the gamepad bridge lets a phone do this over WiFi. WiFi and Bluetooth share the ESP32's one radio, and WiFi degrades the gamepad link, so the portal runs in a setup mode of its own, never while driving. The robot-side calibration it drives comes from `add-live-joint-calibration`; on the bench, `add-desktop-calibrator` does the same over USB.

## What Changes

- **Setup mode on the bridge.**
  - Entered by holding the left button (GPIO0) for 1 s while the boot logo is shown; the bridge restarts into setup mode.
  - Bluetooth is not started. The bridge opens a WPA2 access point with a captive portal, and its screen shows the network name, password, address and a joining QR code.
  - Leaving setup mode is a restart into normal operation.
  - The command link and the e-stop button keep working throughout.
- **The portal as the controller.**
  - While a portal page is open and polling, the bridge reports a connected controller. The page's Stand, Calibrate and Sleep buttons are mode requests. Translate and rotate are always zero.
  - A page that stops polling counts as a disconnected controller, so the robot sleeps.
- **Calibration front-end.**
  - A joint editor for offsets, pulse end points and direction. Changes go to the robot as calibration frames, are resent until the robot's report confirms them, and the page shows only reported values.
  - Export produces `calibration_defaults.cpp` text to copy.
- **Plain web page:** one hand-written HTML file with plain JavaScript and no frameworks or external resources. No UI code is shared with the desktop calibrator.

## Capabilities

### New Capabilities

- `setup-portal`: The bridge's setup mode: entering and leaving it, the radio rule (WiFi or Bluetooth, never both), the access point and captive portal, the portal acting as the controller and calibration front-end, and the screen in setup mode.

### Modified Capabilities

None. `joint-calibration` and `usb-link` come from `add-live-joint-calibration`, and this change is a front-end for them.

## Impact

- **Depends on** `add-gamepad-bridge` and `add-live-joint-calibration` (protocol version 2, calibration frames, the defaults format and its golden file).
- **Code:**
  - `core/setup/` (hardware-free, host-tested): `portal_controller`, `calibration_sync` and `calibration_export`, used by the bridge.
  - `firmware/bridge`: the setup-mode boot path, WiFi access point, DNS and HTTP server, the portal page, and the setup screen.
- **No new dependencies:** WiFi, `WebServer` and `DNSServer` come with the pinned Arduino core; LovyanGFX draws the QR code.
- **Out of scope:** OTA updates, bridge settings (dead zone, remapping, brightness), forgetting paired gamepads (the Bluepad32 console does this), and driving over WiFi.
