# Proposal

## Why

The desktop calibrator reads every byte the robot writes on USB, debug text and link frames mixed, and its decoder and text splitter are only tested on hand-picked inputs. A malformed or hostile byte stream could panic the reader, grow memory, or let text bytes pass as a frame field (or a frame byte reach the log). Native fuzzing checks these properties over inputs nobody wrote by hand, at no runtime cost.

## What Changes

- **Go fuzz targets** for the desktop's link parsers, seeded from `test/golden/protocol_v3.txt`:
  - the frame decoder (`desktop/internal/protocol`): it never panics, keeps a bounded buffer, decodes every golden frame to the golden fields, and every accepted frame re-encodes to the bytes it was decoded from (apart from the reserved payload bytes);
  - the stream splitter (`desktop/internal/link`): on arbitrary bytes it never panics and keeps lines bounded; on text interleaved with golden frames it returns exactly those frames and exactly that text, so text never leaks into a frame and frame bytes never reach the text.
- **Bounded runs:** `mise run fuzz` runs each Go target for a configurable time (default 60 s per target). The seed corpus, and any failing input saved under `testdata/fuzz/`, runs on every `go test`, so `mise run desktop-test` and CI exercise the harnesses without randomness.
- **Bug fixes:** any real bug the fuzzers find is fixed with its failing input kept as a regression case.
- **Documentation:** the root and desktop `CLAUDE.md` list the task; `CHANGELOG.md` records the change.

**Non-goals:**
- the C++ libFuzzer harness for `core/protocol` (deferred, see design.md D1);
- any firmware behaviour or link protocol change: `protocol::VERSION` stays 3 and no board needs flashing;
- fuzzing in CI with random inputs (CI stays deterministic).

## Capabilities

### New Capabilities

- `link-fuzzing`: fuzz testing of the host-side link parsers: the properties each harness checks, its seeds, and how bounded runs and the deterministic seed pass are invoked.

### Modified Capabilities

None. The decoder's behaviour is unchanged unless a fuzzer finds a bug, which is then a fix to the existing `usb-link` requirement "Link frames over USB", not a new requirement.

## Impact

- **New:** fuzz tests in `desktop/internal/protocol` and `desktop/internal/link`; their `testdata/fuzz/` corpora when a failing input is found.
- **Changed:** `mise.toml` (the `fuzz` task), `CLAUDE.md`, `desktop/CLAUDE.md`, `CHANGELOG.md`.
- **Dependencies:** none; Go's native fuzzing ships with the pinned Go 1.27.1.
- **Risk:** low. Test-only; a bug found and fixed in the decoder is covered by the existing golden and unit tests as well as its regression input.
