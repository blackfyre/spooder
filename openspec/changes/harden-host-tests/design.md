# Design

## Context

The host tests are PlatformIO's `native` env in the root `platformio.ini` (`test_build_src = yes`, `src_dir = core`), run by `mise run test` with `pio test -e native` and then the bridge build-check test. `build_src_filter` lists the 25 `core/` sources one by one; today it matches `find core -name '*.cpp'` exactly. CI runs `mise run test` on `ubuntu-24.04`, whose GCC 13 ships the ASan and UBSan runtimes (`libasan8`, `libubsan1` are dependencies of `libgcc-13-dev`). The development host (Fedora 44, GCC 16) does not have `libasan` or `libubsan` installed: a sanitised link fails with `cannot find /usr/lib64/libasan.so.8.0.0`. A trial run of the suite with an extra script adding the flags failed to link every test on that host for this reason.

## Goals / Non-Goals

**Goals:**

- Every native Unity test is compiled and linked with `-fsanitize=address,undefined -fno-sanitize-recover=all`, so the first finding aborts the test program and fails `mise run test`.
- A `core/` source missing from, or a stale entry in, `build_src_filter` fails `mise run test` with the names.

**Non-Goals:**

- Sanitising the firmware builds, the virtual robot or the desktop calibrator.
- Generating `build_src_filter` from the tree; the list stays explicit and is checked.
- The coverage report and the `include/`/`lib/` removal (follow-ups below).

## Decisions

### Flags through a PlatformIO extra script

PlatformIO puts unrecognised `build_flags` on the compile line only, and has no link-flags option, so `-fsanitize=…` in `build_flags` would compile but not link the runtime. A `pre:` extra script for `[env:native]` (`test/sanitizers.py`) appends `-fsanitize=address,undefined`, `-fno-sanitize-recover=all` and `-fno-omit-frame-pointer` to both `CCFLAGS` and `LINKFLAGS`. Frame pointers keep ASan's stack traces readable; no optimisation flag changes.

### Probe the runtime; fall back loudly, never in CI

Before adding the flags, the script links a one-line C++ program with the same flags using the env's compiler. If that fails:

- under CI (`CI` set to `true`, which GitHub Actions always sets), the build fails with a message that the sanitiser runtimes are missing;
- otherwise it prints a warning naming the packages (`libasan libubsan` on Fedora; on Debian/Ubuntu they come with GCC) and builds without sanitisers.

The fallback exists because making the sanitised link mandatory would fail `mise run test`, the gate of every delivery, on the development host until packages are installed, a change outside this repository that an unattended delivery cannot make. CI keeps the guarantee, and the warning makes the weaker local run visible. A `SPOODER_REQUIRE_SANITIZERS=1` variable makes the probe failure fatal outside CI too, for a host that has the runtimes and wants the strict gate.

### Source-list check as a small Python script

`test/check_test_sources.py` (standard library only) reads `platformio.ini` with `configparser`, takes the `+<…>` entries of `[env:native]` `build_src_filter`, lists `core/**/*.cpp` relative to `core/`, and exits non-zero printing `missing from build_src_filter:` and `not in core/:` lines with the names, sorted. Header files, `-<…>` entries and `+<…>` patterns other than plain file paths are out of scope: an entry with a wildcard is reported as an error, so the list stays a plain list the check can compare. `test/test_check_test_sources.py` (`unittest`) covers match, a missing file, an extra entry and a wildcard entry with crafted trees in a temporary directory. `mise run test` runs the test and then the check before `pio test`, as it already runs the bridge's `test_image_size` with plain `python`.

### Fixing findings

Each sanitiser finding in `core/` is fixed at its cause with a Unity regression test that fails under the sanitiser before the fix. A finding in test code only is fixed in the test. No finding is suppressed with an attribute or a suppression file unless it is a false positive, which is then recorded here.

## Risks / Trade-offs

- [The local fallback lets UB pass on a host without the runtimes] → CI fails on it; the warning is printed on every such run; the strict variable exists.
- [ASan slows the tests and raises memory use] → the suite is small (about 30 programs); the slowdown is seconds.
- [UBSan aborts tests that rely on implementation-defined float-to-int conversion] → those are real portability defects on the robot's Arm toolchain too, and are fixed.
- [Verifying the sanitised path on the development host] → run `mise run test` in an `ubuntu:24.04` container with podman (mirroring CI), or rely on CI for the branch.

## Assumptions

- "Keep CI green on ubuntu-24.04" means the sanitised run must pass in CI; the local Fedora host may run unsanitised with a warning until `libasan` and `libubsan` are installed.
- The probe's `CI=true` test is enough to identify CI; no workflow change is needed.
- The checker and its test live in `test/`, next to the tests they guard, rather than in `tools/`, matching `firmware/bridge/scripts/` keeping its checker next to what it checks.

## Follow-ups

Recorded from the brief, not delivered by this change:

1. **Coverage report.** `mise run coverage`: build the native env with `--coverage`, run the tests, and write an HTML and a text report to `build/coverage/` with gcovr (pinned exactly, run through `uv run --with gcovr==<version>` or an inline-dependency script). Report only, no threshold. Mention it in the root `CLAUDE.md` command list.
2. **Remove the root `include/` and `lib/` directories.** They hold only PlatformIO's boilerplate `README` files; nothing in the repository references them (checked with `git grep`, outside `external/` and the archived changes), and PlatformIO does not need them to exist.
