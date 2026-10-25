# Tasks

## 1. Decoder fuzz test

- [x] 1.1 Change `readVectors` in `golden_test.go` to take `testing.TB`, and add `FuzzDecoder` in `desktop/internal/protocol`, seeded with every golden frame from `test/golden/protocol_v3.txt` and the extra seeds of design D2, checking the properties of design D3 (bounded `Pending`, in-order byte accounting, each span decoding alone to the same frame, canonical re-encoding, byte-identical re-encoding of canonical spans). Verify `go test -run FuzzDecoder ./internal/protocol` passes in `desktop/`.
- [x] 1.2 Add a test that `Push` and `Next` allocate nothing on a stream of golden frames. Verify `go test -run Alloc ./internal/protocol` passes.
- [x] 1.3 Add the CRC-valid non-canonical seeds of design D2 to `FuzzDecoder`, and a plain test asserting each is accepted with the expected fields and re-encodes to the canonical bytes. Verify `go test -run 'NonCanonical|FuzzDecoder' ./internal/protocol` passes.

## 2. Split fuzz tests

- [ ] 2.1 Add `FuzzSplitArbitrary` in `desktop/internal/link`, reading the input through `(*Link).read` on an in-memory port (design D4), seeded from the golden frames, checking the properties of design D4: frames equal to a bare decoder's, lines equal one for one to the reference model's, with the line-boundary seeds of design D2. Verify `go test -run FuzzSplitArbitrary ./internal/link` passes in `desktop/`.
- [ ] 2.2 Add `FuzzSplitInterleaved`, building printable text lines and golden frames from the input as in design D4, keeping every text line shorter than `MaxLineLength` bytes, checking that the frames come back in order and the lines come back one for one. Verify `go test -run FuzzSplitInterleaved ./internal/link` passes.

## 3. Bounded runs and found bugs

- [ ] 3.1 Add the `fuzz` mise task of design D5 (`--time`, default `60s`). Verify `mise run fuzz --time 10s` runs every target and exits zero, and that a target made to fail on purpose (temporarily) makes it exit non-zero.
- [ ] 3.2 Run `mise run fuzz` at the default time. For each failing input found, fix the bug in the decoder or the split and commit the input under `testdata/fuzz/` as a regression case; if none is found, record that in this task. Verify `mise run desktop-test` passes with the saved inputs.

## 4. Documentation and gates

- [ ] 4.1 List `mise run fuzz` in the root `CLAUDE.md` commands, name the fuzz tests in `desktop/CLAUDE.md`, and record the change in `CHANGELOG.md` under Unreleased. Verify the three files mention the task.
- [ ] 4.2 Run the gates: `mise run test`, `mise run desktop-test`, `mise run desktop-build`, `mise run e2e`. Verify each exits zero.
