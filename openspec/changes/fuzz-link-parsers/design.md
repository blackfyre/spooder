# Design

## Context

- `desktop/internal/protocol/decoder.go` is the Go copy of `protocol::Decoder`: a fixed `[MaxFrameSize]byte` buffer, `Push`/`Next`/`Idle`, and a `Discard` callback that receives every byte outside a decoded frame (bytes between frames and the start marker of each rejected candidate). `Encode` re-encodes a decoded `Frame`; `Fields` prints it in the golden file's spelling.
- `desktop/internal/link/link.go` `(*Link).read` wires the decoder to `textAccumulator` (`text.go`), which turns discarded bytes into `Line`s of at most `MaxLineLength` input bytes, escaping non-printable bytes as `\xNN`. At end of stream it loops `Idle`/`Next` until nothing is pending, then flushes the partial line. The package already has a fake port (`fake_test.go`) that the link tests read through.
- `desktop/internal/protocol/golden_test.go` already parses `test/golden/protocol_v3.txt` (`readVectors`) and decodes byte by byte (`decodeAll`).
- Re-encoding is canonical, not byte-identical, for some CRC-valid inputs: command payload byte 13 and pose payload byte 1 are reserved and re-encoded as zero; telemetry payload bytes 27 to 31 are unused and re-encoded as zero; telemetry's `outputsOn` (payload byte 2) decodes any non-zero value as true and re-encodes it as 1. The CRC is recomputed, so it differs too. Every golden frame is canonical.
- No clang is installed on the development machine or in mise; mise offers clang only through conda or an asdf plugin that builds LLVM from source.

## Goals / Non-Goals

**Goals:**
- Go native fuzz tests for the decoder and the stream split, seeded from the golden file, checking the properties in the `link-fuzzing` spec.
- `mise run fuzz` for bounded random runs; the seed pass inside `mise run desktop-test`.

**Non-Goals:**
- The C++ libFuzzer harness (D1).
- Fuzzing the session or UI layers above the split.
- Random fuzzing in CI.

## Decisions

### D1. Deliver the Go harnesses; defer the C++ libFuzzer harness

The brief has two independent parts. The Go part needs nothing new and lands in an existing gate. The C++ part needs clang with libFuzzer and the sanitizer runtimes, which no pinned toolchain here provides: mise's clang comes from conda (whose packages split `compiler-rt` and are not a tested libFuzzer source) or an asdf plugin that compiles LLVM for an hour. Pinning a distro clang through apt in CI would break the rule that every build resolves a pinned version. So this change delivers the Go harnesses and records the C++ harness as a follow-up.

Alternative considered: a C++ harness built with the existing host g++ using `-fsanitize=address,undefined` and a hand-written driver replaying the golden frames. It gives sanitizers without coverage-guided search, which is most of the value of the brief's part (b) missing; better done properly in its own change.

### D2. Seeds come from the golden file at test time

Each fuzz test reads `test/golden/protocol_v3.txt` and calls `f.Add` per frame: the protocol package with its existing `readVectors` helper, changed to take `testing.TB` so a `*testing.F` can call it, the link package with a small reader of its own in a `_test.go` file (the frame column only), since test helpers cannot be shared across packages without putting them in the binary. The golden file stays the single source of seeds, so a protocol change regenerating it updates the seeds too. Extra seeds: two frames back to back, a frame with garbage before and after, a truncated frame, and a lone start marker.

### D3. Decoder fuzz properties

`FuzzDecoder` feeds the input byte by byte, calling `Next` after each byte, then drains like `link.read`'s end of stream (loop `Idle` and `Next` while `Pending() > 0`). It keeps a cursor into the input: each `Discard` byte must equal the input byte at the cursor (then the cursor advances by one), and each decoded frame owns the next `HeaderSize + PayloadSize(type) + CRCSize` input bytes from the cursor (its span; the cursor advances past it). It checks:
- no panic (the fuzz engine reports it);
- `Pending() <= MaxFrameSize` after every call;
- at the end the cursor is at the end of the input (every byte accounted for once, in order);
- the span, decoded alone by a fresh decoder, gives one frame equal to the decoded one;
- `Encode(f)` decoded alone gives an equal frame;
- if the span is canonical (see Context), `Encode(f)` equals the span byte for byte. Canonicality is checked by a small test helper listing the non-canonical positions per frame type, not by comparing with `Encode`.

For a golden seed, the existing `TestGoldenVectors` already checks the fields; the fuzz test checks only the generic properties, so it needs no golden-specific branch.

Unbounded allocation is checked by construction plus one assertion: the decoder's state is a fixed array, and a plain test asserts `testing.AllocsPerRun` is zero for `Push`/`Next` on a golden stream (no `Discard`).

### D4. Split fuzz properties

The split is tested by calling `(*Link).read` directly, on a `Link` whose handler records lines and frames and an in-memory port that returns the input in chunks (sizes derived from the input) and then an error, so the end-of-stream drain and line flushing are the real ones. The existing fake port and supervisor are not used: they synchronise through channels and sleeps, far too slow per fuzz iteration. Two targets:
- `FuzzSplitArbitrary`: the input is the stream. Checks no panic; `len(Line.Text) <= 4*MaxLineLength` for every line (a line holds at most `MaxLineLength` received bytes, each rendered as at most four characters); the frames handed to the handler equal those a bare decoder returns for the same input; and the lines' `Stray` counts sum to the number of `0xA5` bytes that bare decoder discards. Exact text recovery is checked by the interleaved target, where the text is printable and so renders unchanged.
- `FuzzSplitInterleaved`: the input is a recipe: bytes are consumed as (choice, length) pairs that pick either an accepted golden frame by index (rejected golden lines are left out of the recipe) or a run of printable ASCII text (no `0xA5`, ending in `\n`) derived from the input bytes. The test writes the stream, reads it through the split, and checks the frames equal the chosen golden frames in order and the lines equal the written text lines.

Text is split at `MaxLineLength`, so the recipe keeps text lines shorter than that, or the check compares the concatenated text instead of line by line.

### D5. The mise task

`mise run fuzz` takes `--time <duration>` (default `60s`) and runs, for each package and target, `go test -run '^$' -fuzz '^FuzzName$' -fuzztime <time> ./internal/<pkg>` in `desktop/` with `CGO_ENABLED=0` where the package allows it (the protocol and link packages do not need Gio). Go runs one fuzz target per invocation, so the task lists the targets explicitly; it stops at the first failure and exits non-zero. A failing input is written by Go to `testdata/fuzz/<Target>/`, which is committed with the fix.

### D6. The gate

`go test ./...` in `mise run desktop-test` already runs every `Fuzz*` function over its seed corpus and `testdata/fuzz/` without random inputs, and CI runs `desktop-test`. No new gate or CI step is needed.

## Risks / Trade-offs

- [The fuzzers find nothing in 60 s] → The value is the standing harness; longer runs are one flag away.
- [A fuzz test reading the golden file breaks if the file moves] → Same path the golden tests already use; they would fail together.
- [The interleaved recipe constrains the input space] → The arbitrary-bytes target covers the rest.
- [The canonical-bytes exception hides a real mismatch] → Only the listed positions are exempt, per frame type; every other accepted span must re-encode byte for byte, and every span must decode alone to the same frame.

## Assumptions

- The brief's part (b) is deferred (D1); its delivery is the follow-up below.
- "Never allocates unboundedly" is satisfied by the fixed decoder buffer, a bounded line length, and the zero-allocation test (D3), not by measuring heap growth inside the fuzz loop.
- "Every accepted frame re-encodes to the same bytes" holds for canonical frames (all golden frames); for the rest the property is a canonical round trip, because the protocol defines reserved and unused bytes as zero and booleans as 0 or 1 but the decoder, like the C++ one, does not reject other values. Rejecting them would be a protocol behaviour change, out of scope.
- No CI change is needed: the seed corpus runs in `desktop-test`, which CI already runs.

## Follow-ups

- **C++ libFuzzer harness** for `core/protocol` (`protocol::Decoder` and the `decode*` functions) and `core/protocol/link_receiver.cpp` (driven with decoded command and calibration-set frames through `accept`, `acceptCalibration`, `processCalibration`, `makeReport`), built with `clang -fsanitize=fuzzer,address,undefined`, seeded from the golden file, with a few seconds per target in a gate. Needs a decision on a pinned clang with libFuzzer (a mise backend that ships `compiler-rt`, or a pinned LLVM release tarball).
- A nightly CI job running `mise run fuzz` for a few minutes, reporting rather than gating, once it is clear the targets are quiet.
