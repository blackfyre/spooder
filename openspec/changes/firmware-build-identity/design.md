## Context

The robot's first console line is `servo profile <name>: joint speed budget <n> deg/s`, printed by `robot_tick` on its first tick through `Hardware::log`. It is usually lost: the RP2040's USB console enumerates a second or two after reset, and `stdio` drops text while no host is connected (`PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000`). The virtual robot runs the same `robot_tick`, its `log` writing to the USB pseudo-terminal, which also drops text while nothing has it open (`Pty::connected()`).

The bridge prints `spooder bridge, Bluepad32 <version>` from `setup()` and, in setup mode (which never runs `setup()`), `spooder bridge: setup mode (Bluetooth off)` from `setup_mode_start`. Its screen shows the logo, then a start-up log of up to eight lines of `status_screen::LINE_CHARS` (28) characters added with `display::bootMessage`, within 3 s.

Three builds compile firmware-side code: the robot's pico-sdk CMake project, the virtual robot's host CMake project, and the bridge's ESP-IDF project driven by PlatformIO (its `main` component is plain CMake). ESP-IDF has its own `PROJECT_VER` from `git describe --tags`, but only at configure time and in a different form.

## Goals / Non-Goals

**Goals:**
- Each firmware names its build where the operator already looks, in one form on every board.
- The identity follows the checkout without a manual reconfigure, is reproducible, and changes cost one small recompile and a relink.
- No protocol change.

**Non-Goals:**
- The desktop calibrator's and `spooder-monitor`'s own identity (follow-up below).
- Showing the robot's build anywhere but its own console; comparing the two boards' builds automatically.
- Version numbers or releases.

## Decisions

### One CMake script for the identity
`cmake/build_identity.cmake`, run in script mode (`cmake -P`) by an always-run custom target in each build, before the target that uses it. It runs `git -C <repository root> describe --always --dirty --abbrev=12`; if git is missing, the command fails or the directory is not a checkout, the identity is `unknown`. It writes a generated C++ source into the build directory:

```cpp
namespace build_identity { extern const char ID[] = "<identity>"; }
```

with `file(CONFIGURE ...)`/`configure_file`, which leaves the file untouched when its content is unchanged. So an unchanged checkout rebuilds nothing; a new commit, or the tree turning dirty or clean, recompiles that one file and relinks. The identity holds only git's output: no time, user or host, so the same checkout gives the same image identity.

*Alternatives:* a compile definition (`-DSPOODER_BUILD_ID=...`) changes the flags of every source and rebuilds everything; computing it only at configure time goes stale after a commit until someone reconfigures; ESP-IDF's `PROJECT_VER` is configure-time only and uses `--tags`, so the two boards would not match in form.

The custom target is always out of date, so it runs `git describe` once per build (tens of milliseconds). `--dirty` covers tracked files only: an untracked or ignored file, such as the git-ignored `firmware/servo2040/src/power_limits.h`, does not mark the build dirty. That is accepted; the limits are reported in the diagnostics frame.

### Wiring in each build
- **Robot** (`firmware/servo2040/CMakeLists.txt`): the generated source is added to `spooder` (not to `spooder_core`, which the host tests share), with the target depending on the custom target.
- **Virtual robot** (`tools/virtual-robot/CMakeLists.txt`): the same, into `virtual-robot`; its identity is the virtual robot binary's own build.
- **Bridge** (`firmware/bridge/main/CMakeLists.txt`): the generated source is listed in `idf_component_register`'s `SRCS` (marked `GENERATED`) and `${COMPONENT_LIB}` depends on the custom target. If PlatformIO's ESP-IDF integration does not run the custom target on every build, the fallback is a PlatformIO pre-script calling the same `cmake -P` before the build; the form and the script stay the same.

### Banner text in core
`core/identity/build_identity.{h,cpp}`, hardware-free and host-tested:
- `banner(out, size, component, id)` writes `spooder <component>: build <id>\n`. An empty or null id becomes `unknown`; any byte of the id outside printable ASCII (0x20 to 0x7E) becomes `?`, and the id is cut at 40 characters, so the line can never put a frame byte (0xA5) or a control character on the robot's USB port.
- `fit(out, width, id)` for the screen: the id as is when it fits `width` characters, otherwise `..` followed by its end, so the abbreviated hash and the `-dirty` mark stay visible.

The generated `build_identity::ID` is declared in the same header but defined only by the generated source of each firmware build; the host tests do not link it and test the functions with literal ids.

### Robot: on the first tick and on each USB connect
`robot_tick::Inputs` gains `usbConnected` (the firmware passes `stdio_usb_connected()`, the virtual robot `board.usb.connected()`). `robot_tick::init` takes the build identity (a new last parameter, defaulting to `"unknown"`, so existing callers and tests stay valid) and stores the pointer in `RobotState`. `tick` logs the banner on its first tick, before the servo profile line, and again on each tick where `usbConnected` turns from false to true. On the board, the first-tick banner is normally dropped and the connect banner is the one seen; a host that opens the port therefore always sees the build first. The servo profile line is unchanged (first tick only). The serial-keys build (`--serial`) prints the banner the same way.

*Alternative:* print it inside the once-a-second status line. Rejected: the leg-actuation spec keeps that line unchanged, and the identity does not change at run time.

The banner goes through `Hardware::log`, so the virtual robot prints it on its USB terminal with no code of its own beyond `usbConnected`.

### Bridge: console at boot, start-up log on screen
`bridge.cpp::setup()` prints `spooder bridge: build <id>` first, then the existing Bluepad32 line, and adds `Build <fitted id>` as the first start-up log line after the logo (`display::bootMessage`; the log then has five lines of its eight). `setup_mode_start` prints the same console line before `spooder bridge: setup mode (Bluetooth off)`. The setup-mode screen is unchanged. The start-up log line is 28 characters: `Build ` leaves 22, enough for 12 hex digits and `-dirty` (18); a longer describe (an annotated tag) is shortened from the start.

The status pages are unchanged, so `mise run bridge-screenshots` output does not change and the screenshots are not regenerated.

## Risks / Trade-offs

- [PlatformIO's ESP-IDF build may not rebuild the generated source when only git state changed] → Checked in the task by committing an empty change between two `mise run build-bridge` runs and reading the identity string in the built image (`strings`); fallback above.
- [CI's shallow checkout has no tags and may lack history] → `--always` gives the hash in a shallow clone; CI's identity is still correct.
- [A banner line on connect changes what the e2e tests and the calibrator's log see first] → It is one more text line; the e2e run is a gate.
- [The robot's banner is lost when the host opens the port but reads only later] → The host has the port open, so the text is buffered by the host's driver; a console opened later sees the next connect banner.

## Assumptions

- The brief's "smallest coherent part" is the two firmwares (and the virtual robot, which shares the robot's tick): a mismatched or stale flash is a board-side question. The desktop tools are a separate, independent part.
- `--abbrev=12` (the brief leaves the length open): 12 hex digits stay unique and fit the bridge's start-up line with `-dirty`.
- Plain `git describe` (annotated tags only, none exist yet), not `--tags`, so a lightweight tag does not change the form.
- The bridge shows the identity on its start-up log (the brief allows the boot logo or an existing page) rather than on the link page, keeping the status pages and their screenshots unchanged.
- The robot repeats the banner on each USB connect, because the first-tick line is normally lost before the console enumerates; this is the reading of "print it in the boot banner" that makes it visible.

## Follow-ups

- **Desktop identity:** embed the desktop calibrator's and `spooder-monitor`'s own build identity (Go `debug.ReadBuildInfo` `vcs.revision`/`vcs.modified`, or `-ldflags -X` from the same describe in `mise run desktop-build`), shown in the calibrator's title or about text and by `spooder-monitor --version`.
- **The robot's build in the calibrator:** needs no protocol change: the connect banner already arrives as a debug text line, so the calibrator could pick `spooder robot: build ...` out of its log and show it beside the connection state. Showing it on the bridge would need a protocol change (a frame carrying the robot's identity) and is out of scope.
