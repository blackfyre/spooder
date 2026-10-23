# desktop/

The desktop calibrator, a Go module (`github.com/blackfyre/spooder/desktop`) with a Gio UI, kept apart so Gio and cgo never reach the firmware builds. Gates: `mise run desktop-test` (gofmt, go vet, go test), `mise run desktop-build` and `mise run e2e`.

**Dependencies.** Gio needs cgo and the Wayland, X11 and EGL development packages on Linux. On Fedora: `sudo dnf install wayland-devel libxkbcommon-devel libxkbcommon-x11-devel libX11-devel libXcursor-devel libXfixes-devel mesa-libEGL-devel vulkan-headers`. `desktop-build` and `desktop-test` run `desktop-deps` first, which prints the `dnf` line for whatever is missing. Opening the robot's port needs membership of the `dialout` group.

- `cmd/spooder-calibrator`: `main`, the wiring.
- `cmd/spooder-monitor` + `internal/console`: the text console behind `mise run monitor`. It uses the link read-only (no source, so no writer) and never sends a frame; only `--keys` writes, printable ASCII only. No Gio, and cgo only on macOS.
- `internal/protocol`: the link protocol in Go, byte for byte the C++ one, tested against `test/golden/protocol_v3.txt`.
- `internal/link`: the serial port (`go.bug.st/serial`), the split of the stream into frames and debug text lines, the 20 ms writer (none for a nil source) and reconnecting by USB serial number.
- `internal/session`: the single state owner: observe or control, the e-stop, mode and clear counters, driving (held keys and walking settings), telemetry, the latest pose, and the ten-minute history with its CSV export.
- `internal/legview`: the legs pane's geometry: forward kinematics (tested against `test/golden/leg_pose.txt`), leg mounting, servo limits; and the whole robot in the body frame with the top, front, left and isometric projections (`views.go`) for the four-view window (`internal/ui/views.go`, its own Gio window); `gravity.go`, the estimated centre of gravity and support polygon that window draws, with a battery toggle (tested against `test/golden/centre_of_gravity.txt`).
- `internal/calib`: the joint table shown from reports, pending edits, one change in flight, resend and confirmation by request id.
- `internal/export`: `calibration_defaults.cpp` in the fixed format (tested against the fixture `test/golden/calibration_defaults.cpp`), the checkout check, the diff and the atomic write.
- `assets/`: the app icon, `icon.svg`, with its rendered PNGs and `icon.ico`.
- `internal/virtualrobot`: runs the virtual robot for tests; `e2e/`: end-to-end tests of the real link and session against it (skipped when it is not built).
- `internal/ui`: the Gio window in the Blackfyre look, with the charts tab (`charts*.go`: ten minutes of telemetry, diagnostics and pose on one time axis; series colours validated per mode); `internal/ui/fonts`: Archivo and JetBrains Mono with their OFL licences. `internal/appearance` and `internal/winreg` are copied from bopen (MIT, `LICENSE.bopen`).

## Rules

- The calibrator never opens a port by itself: it lists ports (the Servo 2040, USB ID 2e8a:000a, preselected; or the `--port` path) and connects on request. Its unit tests use in-memory ports or pseudo-terminals they create; the e2e tests use only the virtual robot's terminal.
- `internal/protocol` and `internal/legview` copy C++ behaviour; keep them in step through the golden files, never by editing a golden file to match.
