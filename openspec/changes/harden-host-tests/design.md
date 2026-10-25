# Design

## Context

The host tests are PlatformIO's `native` env in the root `platformio.ini` (`test_build_src = yes`, `src_dir = core`), run by `mise run test` with `pio test -e native` and then the bridge build-check test. `build_src_filter` lists the 25 `core/` sources one by one; today it matches `find core -name '*.cpp'` exactly. CI runs `mise run test` on `ubuntu-24.04`, whose GCC 13 ships the ASan and UBSan runtimes (`libasan8`, `libubsan1` are dependencies of `libgcc-13-dev`). The development host (Fedora 44, GCC 16) does not have `libasan` or `libubsan` installed: a sanitised link fails with `cannot find /usr/lib64/libasan.so.8.0.0`. A trial run of the suite with an extra script adding the flags failed to link every test on that host for this reason.

## Goals / Non-Goals

**Goals:**

- Every native Unity test is compiled and linked with `-fsanitize=address,undefined,float-cast-overflow -fno-sanitize-recover=all`, so the first finding aborts the test program and fails `mise run test`.
- A `core/` source missing from, a stale entry in, or an entry of any other form in `build_src_filter` fails `mise run test` with the names.

**Non-Goals:**

- Sanitising the firmware builds, the virtual robot or the desktop calibrator.
- Generating `build_src_filter` from the tree; the list stays explicit and is checked.
- The coverage report and the `include/`/`lib/` removal (follow-ups below).

## Decisions

### Flags through a PlatformIO extra script

PlatformIO passes `build_flags` through SCons's `ParseFlags`, which (SCons 4.11.1, the pinned `tool-scons`) already puts `-fsanitize=…` on both the compile and the link line; `-fno-sanitize-recover=all` and `-fno-omit-frame-pointer` go to the compile line only, where they belong. `build_flags` cannot be conditional on the runtimes being present, though, so a `pre:` extra script for `[env:native]` (`test/sanitizers.py`) adds the flags with `env.Append(CCFLAGS=[…], LINKFLAGS=[…])` after the probe: `-fsanitize=address,undefined,float-cast-overflow` on both, `-fno-sanitize-recover=all` and `-fno-omit-frame-pointer` on `CCFLAGS` only. `float-cast-overflow` is named because GCC's `undefined` group does not include it, and an out-of-range float-to-integer conversion is undefined behaviour the kinematics and pulse code could hit. Frame pointers keep ASan's stack traces readable; no optimisation flag changes.

### Probe the runtime; fall back loudly, never in CI

Before adding the flags, the script links a one-line C++ program with the same flags. A `pre:` script runs before the native platform's builder, which deletes `CC`/`CXX` and loads SCons's `gcc` and `g++` tools; SCons's `g++` tool sets `CXX` only when it is absent, so the probe repeats the builder's sequence on a clone of the env (delete `CC` and `CXX`, then `Tool("gcc")` and `Tool("g++")`) and uses that clone's `$CXX`, the same compiler the tests are built with, rather than whatever `CXX` or `c++` holds at that moment. The script prints the probed compiler, so a verbose run shows it next to the compile lines. The selection is a function, `probe_compiler(env)`, that `test/test_sanitizers.py` (`unittest`) checks against real SCons: it loads SCons from PlatformIO's installed `tool-scons` package (under `$PLATFORMIO_CORE_DIR`, default `~/.platformio`, `packages/tool-scons/`), builds an environment whose `CXX` is a different compiler name (`c++`), and asserts that the function returns the compiler the native builder's delete-and-reload sequence selects and leaves the original environment's `CC` and `CXX` unchanged. `mise run test` runs it after `pio test`, which installs the package, and the test fails rather than skips when the package is missing. If the probe fails:

- under CI (`CI` set to `true`, which GitHub Actions always sets) or with `SPOODER_REQUIRE_SANITIZERS=1`, the build fails with a message that the sanitiser runtimes are missing;
- otherwise it prints a warning naming the packages (`libasan libubsan` on Fedora; on Debian/Ubuntu they come with GCC) and builds without sanitisers.

The fallback exists because making the sanitised link mandatory would fail `mise run test`, the gate of every delivery, on the development host until packages are installed, a change outside this repository that an unattended delivery cannot make. CI keeps the guarantee, and the warning makes the weaker local run visible. `SPOODER_REQUIRE_SANITIZERS=1` makes the probe failure fatal outside CI too, for a host that has the runtimes and wants the strict gate.

The decision (probe result and environment → sanitise, warn and continue, or fail, with the message) is a plain function in `test/sanitizer_policy.py`, importable without SCons, which `test/sanitizers.py` calls. `test/test_sanitizer_policy.py` (`unittest`) covers the probe succeeding, and the probe failing locally, with `SPOODER_REQUIRE_SANITIZERS=1` and with `CI=true`, checking the outcome and the message. This makes every branch deterministic on any host, whether or not it has the runtimes.

### Source-list check as a small Python script

`test/check_test_sources.py` (standard library only) reads `platformio.ini` with `configparser`, splits `[env:native]` `build_src_filter` into entries, lists `core/**/*.cpp` relative to `core/`, and exits non-zero printing `missing from build_src_filter:`, `not in core/:` and `invalid entry:` lines with the names, sorted. The filter must have one shape: `-<*>` as its first entry, then only `+<path>` entries whose path is a plain relative path ending in `.cpp` (no wildcard, no `..`). Any other entry, including a second `-<…>` exclusion that would drop a listed source, is an invalid entry, so passing the check means PlatformIO compiles exactly the listed files. Header files are out of scope. `test/test_check_test_sources.py` (`unittest`) covers match, a missing file, an extra entry, a wildcard entry, an additional exclusion and a missing leading `-<*>` with crafted trees in a temporary directory. `mise run test` runs both Python unit tests and then the check before `pio test`, as it already runs the bridge's `test_image_size` with plain `python`.

### Fixing findings

Each sanitiser finding in `core/` is fixed at its cause with a Unity regression test that fails under the sanitiser before the fix. Because `core/` reaches the robot, the bridge and the virtual robot (`core/CLAUDE.md`), any such fix is also verified with `mise run build` (with and without `--serial`), `mise run build-bridge` and `mise run e2e`; a fix that would change the 20 ms tick's timing, a protection decision or the link protocol's bytes is outside this change and is reported instead. A finding in test code only is fixed in the test. No finding is suppressed with an attribute or a suppression file unless it is a false positive, which is then recorded here.

## Risks / Trade-offs

- [The local fallback lets UB pass on a host without the runtimes] → CI fails on it; the warning is printed on every such run; the strict variable exists.
- [ASan slows the tests and raises memory use] → the suite is small (about 30 programs); the slowdown is seconds.
- [UBSan aborts tests that hit an out-of-range float-to-integer conversion] → that is undefined behaviour, a real defect on the robot's Arm toolchain too, and is fixed.
- [Verifying the sanitised path on the development host] → run `mise run test` in an `ubuntu:24.04` container with podman (mirroring CI), or rely on CI for the branch.

## Assumptions

- "Keep CI green on ubuntu-24.04" means the sanitised run must pass in CI; the local Fedora host may run unsanitised with a warning until `libasan` and `libubsan` are installed.
- The probe's `CI=true` test is enough to identify CI; no workflow change is needed.
- PlatformIO 6.2 hides an extra script's standard output below `-vv` and prints compile and link command lines only at `-vvv`, so the script prints the probed compiler on standard output (seen with `pio test -e native -vv` or `-vvv`, next to the command lines) and the warning and the failure on standard error, which `pio test` always shows. Task 2.3's check of the command lines therefore uses `-vvv`. `pio test` builds each test program in its own SCons run, so the probe and the warning repeat once per test program (about 30); each probe is one short link.
- `CI` counts as `true` case-insensitively (`True`, `TRUE`); `SPOODER_REQUIRE_SANITIZERS` only as `1`. Any other value, or none, is local.
- The script skips the probe for a clean target (`pio run -t clean`), which builds nothing.
- The first sanitised run (ubuntu:24.04, GCC 13.3) found one defect, in test code: `test_centre_of_gravity`'s `legMassCentre` kept a pointer into the temporary `LegPoints` that `legInBody` returned (stack-use-after-scope). It is fixed in the test by keeping the `LegPoints` in a local; `core/` had no finding.
- The source filter checked is the one PlatformIO uses: PlatformIO 6.2 appends `PLATFORMIO_BUILD_SRC_FILTER` (or, when that is empty, the old `PLATFORMIO_SRC_FILTER`) to `build_src_filter`, so the checker appends it the same way and any entry it adds is held to the same shape (found in review).
- The checker and its test live in `test/`, next to the tests they guard, rather than in `tools/`, matching `firmware/bridge/scripts/` keeping its checker next to what it checks.

## Follow-ups

Recorded from the brief, not delivered by this change:

1. **Coverage report.** `mise run coverage`: build the native env with `--coverage`, run the tests, and write an HTML and a text report to `build/coverage/` with gcovr (pinned exactly, run through `uv run --with gcovr==<version>` or an inline-dependency script). Report only, no threshold. Mention it in the root `CLAUDE.md` command list.
2. **Remove the root `include/` and `lib/` directories.** They hold only PlatformIO's boilerplate `README` files; nothing in the repository references them (checked with `git grep`, outside `external/` and the archived changes), and PlatformIO does not need them to exist.
