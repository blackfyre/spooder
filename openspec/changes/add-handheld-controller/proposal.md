# Proposal

## Why

The bridge's WiFi drive (`add-wifi-drive`) and the relay of pose and diagnostics frames (`relay-pose-diagnostics`) are merged, but nothing drives the robot through them except the desktop probe. A TrimUI Smart Pro running NextUI (tg5040: Tina Linux, kernel 4.9.191, glibc 2.33, SDL2 2.26.5, 1280×720, two analogue sticks, analogue L2/R2, no L3/R3) is to become Spooder's driving controller, its screen showing more than the bridge's 240×135 screen can. Calibration stays on the bridge's setup portal.

## What Changes

- **A NextUI tool pak**, `Tools/tg5040/Spooder.pak` (`launch.sh`, the `spooder-handheld` binary and the SDL2 add-on library the device lacks), written in Go with gabagool v2 (menus and messages) and go-sdl2 (the live screen). It lives in a nested Go module, `desktop/handheld/`, which reuses the desktop calibrator's Gio-free `internal/protocol` and `internal/session` packages without moving them.
- **Driving.** The sticks, triggers and buttons map to the same desired movement state as the bridge's gamepad mapping (a Go port of `core/gamepad_mapping`, kept in step by a new golden file), with a handheld layer for what the TrimUI lacks (no L3/R3) or reserves (Menu, Power): Select is a modifier, Select + Start toggles the emergency stop, Select + Y held 1 s requests a fault clear and Select + X switches the drive screen to the driving view, a chase camera behind the robot. Command frames (link protocol version 3) go in UDP datagrams to 192.168.4.1:52040 at 50 Hz, from one socket for the whole run, after a 150 ms start-up wait (the obligations of `add-wifi-drive` design D4).
- **One live screen** with the robot's mode, gait, stride, height, speed, gesture, e-stop, faults by name, supply voltage and current, and the WiFi link's own quality: the signal on `wlan0`, robot frames per type and second, datagrams dropped as invalid, lost and late frames by the robot's sequence numbers, and the time since the last frame. No round-trip time is shown: the bridge echoes nothing.
- **Leaving safely.** Menu, and a short or long press of Power, first request Sleep and wait, at most 3 s, for telemetry to confirm it; then the pak exits, suspends or powers off. gabagool's power-button handler gets a hook for this through a patched gabagool (upstream v2.26.6 plus a committed patch, fetched and patched by the build), until upstream has the hook. The pak turns WiFi power saving off for its run and restores the previous setting however it exits, in `launch.sh` and in Go.
- **Joining the bridge.** The user joins `spooder-drive-XXXX` in NextUI's WiFi settings. The pak checks that `wlan0` holds a 192.168.4.x address, renews the lease once (`udhcpc -i wlan0 -n -q -t 5`) if it does not, and otherwise names the network to join.
- **Build and packaging.** A pinned Debian bullseye arm64 image under rootless Podman with qemu-user builds the device binary; the build checks that its highest GLIBC symbol version is at most 2.31. mise tasks fetch the patched gabagool, test, build (desktop Linux amd64 and the device), and package `Spooder.pak` as a zip. CI tests and builds the amd64 binary.
- **A virtual bridge** in the virtual robot: in place of the simulated bridge, it takes the handheld's datagrams on UDP and applies the bridge's own `core/wifi_drive` rules, so the pak's logic runs end to end in `mise run e2e`.
- **Security.** The UDP socket is an outside input: the pak accepts only datagrams from the bridge's address that are exactly one valid robot frame, and counts and drops the rest. Authenticated freshness of command frames stays deferred (design D9).

Deferred to follow-ups (design.md): the other leg views and the centre of gravity, voltage and current over time, the diagnostics screen, the arm64 build in CI, and the upstream gabagool pull request.

## Capabilities

### New Capabilities

- `handheld-controller`: the pak's controls, its screen, the WiFi link handling, the address check, leaving and the power button, power saving, and its build and packaging.

### Modified Capabilities

- `virtual-robot`: a virtual bridge for WiFi drive.

## Impact

- `desktop/handheld/` (new nested Go module): `cmd/spooder-handheld`, the hardware-free packages, the e2e tests, the gabagool patch, `launch.sh`, the container definition and `go.mod`/`go.sum` (gabagool v2.26.6, go-sdl2 v0.4.40, both released more than 7 days ago).
- `test/golden/gamepad_mapping.txt` and a C++ host test reading it; no `core/` source changes.
- `tools/virtual-robot/`: the virtual bridge, built from `core/wifi_drive`.
- `mise.toml` (handheld tasks; `e2e` gains the handheld's tests), `.github/workflows/ci.yml` (a handheld job), `.gitignore`.
- `desktop/CLAUDE.md`, `tools/CLAUDE.md`, the `spooder-hardware-check` skill (installing the pak, joining the bridge, the on-device checks), `Readme.md`, `CHANGELOG.md`.
- No protocol change and no firmware change.
