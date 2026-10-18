# Tasks

## 1. Read-only link

- [x] 1.1 `internal/link`: a nil `Source` runs the reader only (design D2). Verify `go test ./internal/link`: with a nil source nothing is written, frames and lines still arrive, and reconnect still works.

## 2. Monitor

- [x] 2.1 `internal/console`: flags, the handler (text, frame and status lines, throttle, rejection lines, totals), port selection with the pseudo-terminal fallback, the write-refusing port wrapper and key forwarding (design D1, D3–D5). Verify `go test ./internal/console` with in-memory ports: interleaved text and frames, partial reads, a stray 0xA5, golden-vector decode lines, throttling, never-writes, key filtering, reconnect, and a real pseudo-terminal on Linux.
- [x] 2.2 `cmd/spooder-monitor` wiring (signals, terminal restore). Verify `CGO_ENABLED=0 go build ./cmd/spooder-monitor` and `--help` naming the observe-only rule.

## 3. Tasks and documentation

- [x] 3.1 `mise.toml`: `monitor` runs the new monitor with its flags, `monitor-raw` keeps the old console (design D6). Verify `mise tasks` lists both and `mise run monitor --help` shows the flags.
- [x] 3.2 Update `CLAUDE.md` and `spooder-hardware-check` (monitor mentions, `--keys` for the `--serial` build). Verify every task and flag mentioned exists (`grep`).
- [x] 3.3 Run `mise run desktop-test` and `openspec validate add-robot-console-monitor`, and record a sample transcript against a fake port fed with `test/golden/protocol_v2.txt`. Verify all pass.
- [ ] 3.4 On the board (requires the user): `mise run monitor` and `mise run monitor --frames` on a default build, unplug and replug, and `mise run monitor --keys` on a `--serial` build.
