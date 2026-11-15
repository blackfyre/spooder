# Proposal

## Why

`spooder-firmware-shape` asks for mode-dependent behaviour as one `enum class` with one transition function and its guards, no flags that duplicate a state and no sentinels standing for a state. The firmware adapters and tools break that rule in ten places. The worst cluster is the bridge's robot link (`firmware/bridge/main/link.cpp`):

- **The boot mode is encoded four ways.** `drive_mode::Mode` decides it, `bridge_boot_mode_select` translates it into the C enum `bridge_boot_mode` through a `switch` whose `default:` silently becomes Bluetooth drive (`wifi_drive.cpp:49-57`), `app_main` treats every value that is not Bluetooth or setup as WiFi drive (`main.c:37-42`), and `robot_link` re-encodes it as two independent bools, `portalMode` and `handheldMode`, set by three start functions (`link.cpp:36-44, 221-240`) and tested in five places. Nothing stops both bools being true, and a fourth drive mode would compile and fall into Bluetooth drive.
- **The effective emergency stop is computed in three places.** The link task ORs the GPIO35 toggle with the handheld's flag for the command frame (`link.cpp:78-85`); the WiFi drive screen ORs them again from two separately locked snapshots, `robot_link::status()` and `robot_link::handheldView()` (`wifi_drive.cpp:202-207`), so the screen can combine a toggle and a flag that never coexisted; the virtual robot's WiFi bridge writes the same formula a third time (`tools/virtual-robot/src/wifi_bridge.cpp:171`).
- **The receive idle gap lives in function statics.** `receive()` keeps `lastByteMs` and `idleHandled` as `static` locals (`link.cpp:155-183`), a hidden two-state machine.

This change removes those three. The other seven items of the brief (the bridge display phase, servo output, setup mode's exit and redraw flags, the Bluetooth controller pointer and the three virtual-robot items) are independent of the link and are recorded as follow-ups in design.md.

## What Changes

- `robot_link` starts through one `robot_link::start(drive_mode::Mode)`; `startPortal()` and `startHandheld()` go. The link keeps the one mode it was started in and branches on it with exhaustive `switch` statements without `default:`, so a new drive mode fails the build until the link handles it.
- `bridge_boot_mode` stays only as the C mirror `app_main` needs: each value is pinned to its `drive_mode::Mode` with `static_assert`, `bridge_boot_mode_select` converts without a `default:` fallback, and `app_main` dispatches with an exhaustive `switch`.
- The emergency stop the command frames carry is computed in one function of the link, under the lock, from one shared core rule (`handheld_controller::commandStop`, host-tested) that the virtual robot's WiFi bridge also uses. `robot_link::Status::emergencyStop` becomes that value, so the screen and the portal read the same stop the frames carry, from one snapshot; `HandheldView::emergencyStop` and the screen's own OR go.
- The receive idle gap becomes file-scope explicit state in the link: one `enum class` with its transition function, no function statics.
- Behaviour is unchanged, except that the WiFi drive screen's emergency stop now always matches one consistent link state (it could briefly combine two snapshots).
- `CHANGELOG.md`, `firmware/bridge/CLAUDE.md` and the firmware-shape skill name the new entry point.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `status-display`: the emergency stop on the screen is the one the command frames carry, read with the rest of the link state in one snapshot.
- `bridge-build`: every bridge boot mode has exactly one boot path, checked at compile time.

## Impact

- `firmware/bridge/main/link.{h,cpp}`, `wifi_drive.{h,cpp}`, `main.c`, `bridge.cpp`, `setup_mode.cpp`.
- `core/wifi_drive/handheld_controller.{h,cpp}` (one function) and `test/test_handheld_controller`; a comment fix in `core/status/status_screen.h`.
- `tools/virtual-robot/src/wifi_bridge.cpp` (one call).
- No link protocol change; the bridge must be reflashed to carry the change, the robot need not be.
