# Tasks

## 1. Start check

- [x] 1.1 Add the network-name pattern and the start check to `internal/wlan` (D1), returning the network started on, and one `iw dev wlan0 link` read giving the network and the signal level (D3). Verify with `go test ./internal/wlan` the joined, stale lease and not joined cases, that `spooder-1a2b`, `Home WiFi`, a name with upper-case or five digits, and no network fail without `udhcpc`, and that the pattern test fails with the name check removed.

## 2. Inactive off the network

- [x] 2.1 Let a `udplink` step hold a tick unsent (D2). Verify with `go test ./internal/udplink` that a held tick writes nothing and does not advance the sequence number.
- [x] 2.2 Keep the network started on in `internal/app`, sample the network with the signal level, hold every tick while off it, rejoin as a resume does, and put the state in the screen model (D2, D3, D4). Verify with `go test ./internal/app` that leaving the network stops the frames, a failed read keeps the verdict, another `spooder-drive-` network stays inactive, rejoining sends Sleep with a held control masked and rebaselines, a leave off the network acts after 3 s, and nothing changes off the device.
- [x] 2.3 Draw the `OFF NETWORK` chip and the rejoin notice (`cmd/spooder-handheld/screen.go`). Verify with `go test ./cmd/spooder-handheld` the chip and the notice, and save an off-network screenshot.

## 3. Documentation and gates

- [x] 3.1 Update `docs/handheld.md`, `desktop/CLAUDE.md` (`internal/wlan`, `internal/udplink`), the `spooder-hardware-check` skill's handheld checks and `CHANGELOG.md` under Unreleased. Verify by reading them against the spec delta.
- [x] 3.2 Run `mise run lint`, `mise run test`, `mise run desktop-test`, `mise run e2e`, `mise run handheld-test`, `mise run handheld-build` and `mise run handheld-package`. Verify all pass.
- [ ] 3.3 On the TrimUI with the bridge in WiFi drive: start the pak on the bridge's setup network and on another network, and check it names them and sends nothing; drive, switch NextUI's WiFi to another network, and check the robot sleeps and the screen shows `OFF NETWORK`; switch back and check it requests Sleep and drives again after Y.
