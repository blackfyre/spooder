# Development

## Verification

```sh
mise run lint              # C++ formatting and clang-tidy on core/
mise run test              # host-side Unity tests of core/, and the bridge and robot build checks' tests
mise run build             # robot firmware (also with --serial: USB keys instead of the link)
mise run build-bridge      # bridge firmware, with its image-size check
mise run desktop-test      # gofmt, go vet, staticcheck and go test of the calibrator
mise run desktop-build     # the calibrator and the WiFi drive probe
mise run e2e               # the calibrator's link and session, and the handheld's run loop, against the virtual robot
mise run handheld-test     # the handheld's Go checks (SDL2 development packages; mise run handheld-deps names any missing)
mise run handheld-build    # the handheld for desktop Linux
mise run handheld-package  # the handheld's device build and Spooder.pak.zip
mise run cad               # when hardware/spooder-electronics-mount/ changes
```

These are the verification gates; `CLAUDE.md` lists them with the rules for warnings. CI runs them on a clean Ubuntu runner, except `cad` and `handheld-package`. Hardware behaviour (timing, wiring, power) can only be confirmed on the boards; the staged bring-up is in [`.claude/skills/spooder-hardware-check`](../.claude/skills/spooder-hardware-check/SKILL.md).

## Layout

| Path | What |
|---|---|
| `core/` | Hardware-free C++ shared by both firmwares, the virtual robot and the host tests: movement, protection, protocol, the control tick, bench simulation, gamepad mapping, status, setup mode, WiFi drive |
| `firmware/servo2040/` | The robot: pico-sdk CMake project |
| `firmware/bridge/` | The bridge: ESP-IDF PlatformIO project, including the portal page |
| `desktop/` | The desktop calibrator, the console monitor and the WiFi drive probe, a separate Go module; `desktop/handheld/`, the handheld's pak, a nested one |
| `tools/virtual-robot/` | The virtual robot |
| `test/` | Unity host tests; `test/golden/` holds the fixtures other implementations test against |
| `hardware/` | Spooder's own printed parts (the electronics mount) and reference files |
| `openspec/` | Specifications and changes: the source of truth for intended behaviour |
| `docs/` | This documentation and its images |

Each area has its own `CLAUDE.md` with its layout and rules.

## Screenshots

- The calibrator's are taken by hand against the virtual robot (`mise run desktop-sim`), in both looks.
- The bridge screen's: `mise run bridge-screenshots` writes `docs/images/bridge-*.png`.
- The handheld's: run the drawing test with `SPOODER_SCREENSHOTS` set to a directory, then copy the states shown here (`join`, `driving`, `driveview`, `simulation`, `fault`, `leaving`, `offnetwork`, `gestures`) to `docs/images/handheld-<state>.png`:

  ```sh
  cd desktop/handheld
  SPOODER_SCREENSHOTS=/path/to/dir go test -count=1 -run TestDrawsEveryState ./cmd/spooder-handheld/
  ```
