# Proposal

## Why

`mise run test` is the gate every change to `core/` passes, but it misses two classes of defect. The native Unity tests build without sanitisers, so undefined behaviour (signed overflow, out-of-range float-to-integer conversion, misaligned access) and memory errors (out-of-bounds reads, use after scope) pass silently whenever the result happens to look right. And the `build_src_filter` list in `platformio.ini` that names the `core/` sources under test is maintained by hand, so a new `.cpp` file left out of it is never compiled by the host tests: its tests fail to link only if they exist, and its code goes untested otherwise.

## What Changes

- The native test env builds and runs every Unity test with AddressSanitizer and UndefinedBehaviorSanitizer (`-fsanitize=address,undefined,float-cast-overflow -fno-sanitize-recover=all`, on both the compile and the link; GCC's `undefined` group leaves out `float-cast-overflow`), so any finding fails `mise run test`. Defects the sanitisers find are fixed, each with a regression test.
- Where the compiler cannot link the sanitiser runtimes (a Fedora host without `libasan`/`libubsan`), `mise run test` prints a warning naming the packages and runs the tests unsanitised; under CI (`CI=true`) or with `SPOODER_REQUIRE_SANITIZERS=1` it fails instead, so CI always runs sanitised.
- `mise run test` first checks that the `build_src_filter` list of `[env:native]` is `-<*>` followed by one plain `+<…>` entry per `.cpp` file under `core/`, and fails naming each missing file, each extra entry and each entry of any other form.
- `test/CLAUDE.md` and `CHANGELOG.md` describe both.

The brief's coverage report task and the removal of the root `include/` and `lib/` boilerplate directories are independent of this hardening and are left as follow-ups (see `design.md`).

## Capabilities

### New Capabilities

- `host-tests`: guarantees of the host-side test gate for `core/`: sanitised test runs and a complete source list.

### Modified Capabilities

None.

## Impact

- `platformio.ini` (`[env:native]`: an `extra_scripts` entry), a new sanitiser extra script with unit tests of its policy and its compiler selection, and a source-list check with its unit test, under `test/`.
- `mise.toml`: `mise run test` runs the policy and source-list unit tests and the source-list check before the Unity tests, and the compiler-selection test after them.
- `core/` only where a sanitiser finds a real defect.
- `test/CLAUDE.md`, `CHANGELOG.md`.
- No firmware build, protocol or runtime behaviour changes, except fixes of defects the sanitisers find.
