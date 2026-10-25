# Tasks

## 1. Source-list check

- [ ] 1.1 Add `test/check_test_sources.py` (reads `[env:native]` `build_src_filter`, compares its `+<…>` entries with `core/**/*.cpp`, prints sorted missing and extra names, rejects wildcard entries) and verify `python test/check_test_sources.py` exits 0 on the current tree
- [ ] 1.2 Add `test/test_check_test_sources.py` covering match, a missing source, a stale entry and a wildcard entry on crafted trees; verify `python -m unittest -v test_check_test_sources` passes in `test/`
- [ ] 1.3 Run the test and the check from `mise run test` before `pio test`; verify `mise run test` passes, and fails naming the file after temporarily adding an unlisted `core/` `.cpp` (then remove it)

## 2. Sanitised host tests

- [ ] 2.1 Add the `pre:` extra script `test/sanitizers.py` to `[env:native]` (runtime probe, flags on `CCFLAGS` and `LINKFLAGS`, CI and `SPOODER_REQUIRE_SANITIZERS` handling, warning otherwise); verify `mise run test` on the development host without the runtimes prints the warning and passes, and fails with `SPOODER_REQUIRE_SANITIZERS=1`
- [ ] 2.2 Run `mise run test` sanitised in an `ubuntu:24.04` container (podman, mise toolchains, `CI=true`) and fix every finding at its cause, each with a regression test; verify the sanitised run passes there
- [ ] 2.3 Temporarily add a signed-overflow and an out-of-bounds read to one test, verify the sanitised run fails on each with the sanitiser's report, then revert

## 3. Documentation

- [ ] 3.1 Describe the sanitisers, the local fallback and the source-list check in `test/CLAUDE.md`, and add a `CHANGELOG.md` Unreleased entry; verify `openspec validate harden-host-tests` passes
