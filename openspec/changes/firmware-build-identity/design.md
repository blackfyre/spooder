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

The identity is escaped for the string literal first (`\` and `"` get a backslash): an annotated tag may legally contain `"`, which would otherwise break the generated source. The file is written with `file(CONFIGURE ...)`/`configure_file`, which leaves the file untouched when its content is unchanged. So an unchanged checkout rebuilds nothing; a new commit, or the tree turning dirty or clean, recompiles that one file and relinks. The identity holds only git's output: no time, user or host, so the same checkout gives the same image identity.

*Alternatives:* a compile definition (`-DSPOODER_BUILD_ID=...`) changes the flags of every source and rebuilds everything; computing it only at configure time goes stale after a commit until someone reconfigures; ESP-IDF's `PROJECT_VER` is configure-time only and uses `--tags`, so the two boards would not match in form.

The custom target is always out of date, so it runs `git describe` once per build (tens of milliseconds). `--dirty` covers tracked files only: an untracked or ignored file, such as the git-ignored `firmware/servo2040/src/power_limits.h`, does not mark the build dirty. That is accepted; the limits are reported in the diagnostics frame.

### Wiring in each build
- **Robot** (`firmware/servo2040/CMakeLists.txt`): the generated source is added to `spooder` (not to `spooder_core`, which the host tests share), with the target depending on the custom target.
- **Virtual robot** (`tools/virtual-robot/CMakeLists.txt`): the same, into `virtual-robot`; its identity is the virtual robot binary's own build.
- The script takes the repository root and the output path as `-D` variables, so its checks can run it against disposable fixture repositories in a scratch directory instead of the delivery branch.
- **Bridge** (`firmware/bridge/main/CMakeLists.txt`): the generated source (`${CMAKE_BINARY_DIR}/build_identity_id.cpp`, PlatformIO's build directory) is listed in `idf_component_register`'s `SRCS`, marked `GENERATED` outside ESP-IDF's script-mode requirements pass. PlatformIO's ESP-IDF builder compiles the component sources itself with SCons, from CMake's code model, and never builds CMake targets, so a custom target would never run: the fallback is the one used, a PlatformIO pre-script (`firmware/bridge/scripts/build_identity.py`) running the same `cmake -P` (the platform's own CMake) before every build; the form and the script stay the same.

### Banner text in core
`core/identity/build_identity.{h,cpp}`, hardware-free and host-tested:
- `banner(out, size, component, id)` writes `spooder <component>: build <id>\n`. An empty or null id becomes `unknown`; any byte of the id outside printable ASCII (0x20 to 0x7E) becomes `?`, and the id is cut at 40 characters, so the line can never put a frame byte (0xA5) or a control character on the robot's USB port.
- `fit(out, width, id)` for the screen: the id as is when it fits `width` characters, otherwise `..` followed by its end, so the abbreviated hash and the `-dirty` mark stay visible.

The generated `build_identity::ID` is declared in the same header but defined only by the generated source of each firmware build; the host tests do not link it and test the functions with literal ids.

### Robot: on the first tick and on each USB connect
`robot_tick::Inputs` gains `usbConnected` (the firmware passes `stdio_usb_connected()`, the virtual robot `board.usb.connected()`). `robot_tick::init` takes the build identity (a new last parameter, defaulting to `"unknown"`, so existing callers and tests stay valid) and stores the pointer in `RobotState`. `tick` logs the banner on its first tick, before the servo profile line, and again on each tick where `usbConnected` is true after a tick where it was false. The first tick records `usbConnected` as the previous state, so a first tick with a host already connected prints one banner, not two. The banner is the first line `tick` logs on that tick, before any other text. On the board, the first-tick banner is normally dropped and the connect banner is the one seen.

The connection is sampled once per tick, so the banner follows an observed transition, not every open: a host that closes and reopens the port within one tick (20 ms), or a second program opening the virtual robot's terminal while another holds it, gets no new banner. Delivery is as for every debug line: `Hardware::log` does not report failure, and the board's `printf` (bounded by `PICO_STDIO_USB_STDOUT_TIMEOUT_US`) or the virtual terminal's bounded backlog may drop it. Both are accepted: the identity does not change at run time, and the next connect repeats it. No wait or host request is added, so usb-link timing and the robot-console monitor's observe-only rule are untouched. The servo profile line is unchanged (first tick only). The serial-keys build (`--serial`) prints the banner the same way.

*Alternative:* print it inside the once-a-second status line. Rejected: the leg-actuation spec keeps that line unchanged, and the identity does not change at run time.

The banner goes through `Hardware::log`, so the virtual robot prints it on its USB terminal with no code of its own beyond `usbConnected`.

### Bridge: console at boot, start-up log on screen
`bridge.cpp::setup()` prints `spooder bridge: build <id>` first, then the existing Bluepad32 line. `display::setup()` already queues `Boot` as the first start-up log line, so it takes the build line as a parameter and queues `Build <fitted id>` before `Boot` (the log then has six lines of its eight). `setup_mode_start` prints the same console line before `spooder bridge: setup mode (Bluetooth off)`. The setup-mode screen is unchanged. The start-up log line is 28 characters: `Build ` leaves 22, enough for 12 hex digits and `-dirty` (18); a longer describe (an annotated tag) is shortened from the start.

The status pages are unchanged, so `mise run bridge-screenshots` output does not change and the screenshots are not regenerated.

## Risks / Trade-offs

- [PlatformIO's ESP-IDF build may not rebuild the generated source when only git state changed] → Checked in the task by turning the tree dirty (a temporary edit of a tracked file, restored afterwards) between two `mise run build-bridge` runs and reading the identity string in the built image (`strings`); fallback above. The same check runs for the robot and the virtual robot.
- [CI's shallow checkout has no tags and may lack history] → `--always` gives the hash in a shallow clone; CI's identity is still correct.
- [A banner line on connect changes what the e2e tests and the calibrator's log see first] → It is one more text line; the e2e run is a gate, and a new e2e assertion checks the banner itself.
- [The banner is dropped or a reopen is missed] → See the robot decision above: sampled once per tick and best-effort like every debug line; the next connect repeats it.
- [Cost in the 20 ms tick] → One line of at most 64 characters through the same `Hardware::log` path as the existing status and fault lines, once per connect; no new wait. The tick-time diagnostics and the e2e run cover it.

## Assumptions

- The brief's "smallest coherent part" is the two firmwares (and the virtual robot, which shares the robot's tick): a mismatched or stale flash is a board-side question. The desktop tools are a separate, independent part.
- `--abbrev=12` (the brief leaves the length open): 12 hex digits stay unique and fit the bridge's start-up line with `-dirty`.
- Plain `git describe` (annotated tags only, none exist yet), not `--tags`, so a lightweight tag does not change the form.
- The bridge shows the identity on its start-up log (the brief allows the boot logo or an existing page) rather than on the link page, keeping the status pages and their screenshots unchanged.
- The robot repeats the banner on each USB connect, because the first-tick line is normally lost before the console enumerates; this is the reading of "print it in the boot banner" that makes it visible.
- `cmake/build_identity.cmake` runs `git` from `PATH` (an optional `-DGIT_EXECUTABLE` overrides it) through `execute_process` rather than `find_program`, so a `PATH` without git gives `unknown` instead of a system path being searched. It compares and writes the file itself rather than with `file(CONFIGURE)`, so `@` or `${` in a tag cannot be substituted.
- `fit` applies the banner's printable rule and `unknown` too (the screen font has no glyphs outside ASCII) but not the 40-character cut, so a long identity still ends in its hash and `-dirty` on screen. `BANNER_SIZE` (64) holds the longest banner, `spooder bridge: build ` with 40 characters and the newline.
- The virtual robot's e2e banner test opens the USB terminal with a plain `open`, as a console does, and waits 100 ms (several ticks) after `plug` before opening it again. The calibrator's `link.Serial` opens a pseudo-terminal twice (the first open fails for want of modem lines and is closed), so on the virtual robot the banner can go to that first open and be lost; this is the accepted reopen-within-one-tick case, not a robot defect. On the board, raising DTR succeeds on the first open.

## Follow-ups

- **Desktop identity:** embed the desktop calibrator's and `spooder-monitor`'s own build identity (Go `debug.ReadBuildInfo` `vcs.revision`/`vcs.modified`, or `-ldflags -X` from the same describe in `mise run desktop-build`), shown in the calibrator's title or about text and by `spooder-monitor --version`.
- **The robot's build in the calibrator:** needs no protocol change: the connect banner already arrives as a debug text line, so the calibrator could pick `spooder robot: build ...` out of its log and show it beside the connection state. Showing it on the bridge would need a protocol change (a frame carrying the robot's identity) and is out of scope.
