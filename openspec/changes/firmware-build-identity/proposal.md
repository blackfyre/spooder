## Why

Nothing the robot or the bridge prints says which build is running. A board flashed from an old checkout, from the other side of a protocol change, or with uncommitted edits looks the same as a current one; the only sign of a mismatched pair is a link that looks lost. Each firmware should name its build where the operator already looks: the robot's USB console and the bridge's console and screen.

## What Changes

- **Build identity.** Each firmware build carries an identity derived from git at build time: `git describe --always --dirty --abbrev=12` of the checkout, or `unknown` outside a git checkout (a tarball) or without git. One shared CMake script computes it, escapes it for a C++ string literal, and rewrites a one-line generated source only when the identity changes, so an unchanged build rebuilds nothing and a new commit recompiles one file and relinks. No timestamps or host names: the same checkout gives the same identity.
- **Robot banner.** The robot prints `spooder robot: build <identity>` on the USB console on its first control tick and again on each tick where it sees a USB host connected after none was, so a console or the desktop calibrator opened after boot still sees it. Printable ASCII only.
- **Virtual robot.** Prints the same banner, with its own build's identity, on its USB terminal (it shares the robot's control tick).
- **Bridge.** Prints `spooder bridge: build <identity>` on its serial console at boot, in normal and in setup mode, and shows `Build <identity>` as the first line of the start-up log on its screen.
- No link protocol change; `protocol::VERSION` stays 3. Each board is flashed on its own.
- Not in this change (follow-ups in design.md): the desktop calibrator's and `spooder-monitor`'s own build identity.

## Capabilities

### New Capabilities

- `build-identity`: how a firmware build's identity is derived, and where the robot, the virtual robot and the bridge show it.

### Modified Capabilities

- `status-display`: the start-up log includes the bridge's build identity.

## Impact

- New `cmake/build_identity.cmake` (the git query and the generated source), used by `firmware/servo2040/CMakeLists.txt`, `tools/virtual-robot/CMakeLists.txt` and `firmware/bridge/main/CMakeLists.txt`.
- New hardware-free `core/identity/build_identity` (the banner text, made printable and bounded), host-tested; listed in the root `platformio.ini` and the three builds.
- `core/robot/robot_tick`: the banner on the first tick and on each USB connect (a new `usbConnected` input); `firmware/servo2040/src/main.cpp` and `tools/virtual-robot/src/main.cpp` feed it. `test/test_robot_tick` updated (the banner now precedes the servo profile line); a new desktop e2e assertion that opening the virtual robot's USB terminal yields the banner.
- `firmware/bridge/main/bridge.cpp`, `setup_mode.cpp` and `display.{h,cpp}`: the console line, and the start-up log line queued before the existing `Boot` line.
- Documentation: the `spooder-hardware-check` skill (what the robot and bridge consoles and the start-up log show) and `CHANGELOG.md`. The bridge screenshots show only the status pages, which do not change.
