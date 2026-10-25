# Tasks

## 1. Source-list check

- [x] 1.1 Add `test/check_test_sources.py` (reads `[env:native]` `build_src_filter`, requires `-<*>` first and then only plain `+<….cpp>` entries, compares them with `core/**/*.cpp`, prints sorted missing, extra and invalid entries) and verify `python test/check_test_sources.py` exits 0 on the current tree
- [x] 1.2 Add `test/test_check_test_sources.py` covering match, a missing source, a stale entry, a wildcard entry, an additional `-<…>` exclusion and a missing leading `-<*>` on crafted trees, each failing case asserting the exit status and the named entry; verify `python -m unittest -v test_check_test_sources` passes in `test/`
- [x] 1.3 Run the test and the check from `mise run test` before `pio test`; verify `mise run test` passes, and fails naming the file after temporarily adding an unlisted `core/` `.cpp` (then remove it)

## 2. Sanitised host tests

- [x] 2.1 Add `test/sanitizer_policy.py` (probe result, `CI`, `SPOODER_REQUIRE_SANITIZERS` → sanitise, warn, or fail, with the message) and `test/test_sanitizer_policy.py` covering probe success, local failure (warning naming `libasan libubsan`), failure with `SPOODER_REQUIRE_SANITIZERS=1` and failure with `CI=true` (fatal, message names the missing runtimes); verify `python -m unittest -v test_sanitizer_policy` passes in `test/` and runs from `mise run test`
- [x] 2.2 Add the `pre:` extra script `test/sanitizers.py` to `[env:native]` (probe on a clone of the env with `CC`/`CXX` deleted and the `gcc` and `g++` tools reloaded, as the native builder does, printing the probed compiler; the policy; `-fsanitize=address,undefined,float-cast-overflow` on `CCFLAGS` and `LINKFLAGS`, `-fno-sanitize-recover=all -fno-omit-frame-pointer` on `CCFLAGS`); add `test/test_sanitizers.py` (real SCons from PlatformIO's `tool-scons`, an environment with `CXX=c++`: `probe_compiler` returns the native builder's compiler and leaves the original `CC`/`CXX` unchanged; fails when the package is missing), run from `mise run test` after `pio test`; verify `python -m unittest -v test_sanitizers` passes in `test/`, and `mise run test` on the development host without the runtimes prints the warning and passes, and fails with `SPOODER_REQUIRE_SANITIZERS=1` and with `CI=true`
- [x] 2.3 Run `mise run test` sanitised in an `ubuntu:24.04` container (podman, mise toolchains, `CI=true`), check with `pio test -e native -v` that the compile and link lines carry the sanitiser flags and use the compiler the probe printed, and fix every finding at its cause, each with a regression test; verify the sanitised run passes there, and, if `core/` changed, that `mise run build` (with and without `--serial`), `mise run build-bridge` and `mise run e2e` pass
- [x] 2.4 Temporarily add a signed overflow, an out-of-range float-to-integer conversion and an out-of-bounds read to one test, verify the sanitised run fails on each with the sanitiser's report, then revert

## 3. Documentation

- [x] 3.1 Describe the sanitisers, the local fallback and the source-list check in `test/CLAUDE.md`, and add a `CHANGELOG.md` Unreleased entry; verify `openspec validate harden-host-tests` passes
