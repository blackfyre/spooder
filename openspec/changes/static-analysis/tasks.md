# Tasks

## 1. Pinned tools and the lint task

- [ ] 1.1 Pin `"pipx:clang-format" = "22.1.8"` and `"pipx:clang-tidy" = "22.1.8"` in `mise.toml`, and add the `lint` task running `tools/lint.sh`, which lists Spooder's own C++ files with `git ls-files` (the checked directories, `*.c`, `*.cpp`, `*.h`, `*.hpp`) and runs the formatting check and clang-tidy, failing if either fails. Verify `mise install` provides both tools and `clang-format --version` and `clang-tidy --version` report 22.1.8.

## 2. C++ formatting

- [ ] 2.1 Write `.clang-format` (LLVM base, Allman braces, two-space indent, unindented namespaces) and `.clang-format-ignore` (`external/`, `managed_components/`, `firmware/bridge/main/fonts/`, `test/golden/`, `core/movement/calibration_defaults.cpp`). Pick each remaining option by the fewest changed lines over the file set, and record the chosen values and the changed-line count in design.md. Verify `clang-format --dry-run` over the checked files reports no file under an excluded path.
- [ ] 2.2 Make `tools/lint.sh` run `clang-format --dry-run --Werror` over the listed files. Verify a deliberately misindented line in a `core/` file makes `mise run lint` fail and name the file without modifying it, and the same edit in `external/` or `core/movement/calibration_defaults.cpp` does not.
- [ ] 2.3 Apply `clang-format -i` to the checked files in a commit of its own (layout only), with `// clang-format off`/`on` around data tables that would churn, and add that commit's hash to `.git-blame-ignore-revs`. Verify the formatting check passes, and `mise run test`, `mise run build` (with and without `--serial`), `mise run build-bridge`, `mise run virtual-robot`, `mise run bridge-screenshots` (images unchanged) and `mise run e2e` pass.

## 3. clang-tidy on the core

- [ ] 3.1 Write `.clang-tidy` with the check set in design.md (`WarningsAsErrors: '*'`, header filter on `core/`), and make `tools/lint.sh` write the host compile database with `pio run -e native -t compiledb` and run clang-tidy on each `core/` source in it. Verify a deliberate `float` to `int` narrowing in a `core/` source makes `mise run lint` fail with the file, line and check.
- [ ] 3.2 Fix the existing findings in `core/` without changing behaviour, or suppress each with `NOLINTNEXTLINE(check)` and a reason; disable a check that is mostly noise in `.clang-tidy` with a comment giving the reason. Verify `mise run lint` passes, and `mise run test` (with the golden-vector tests), both firmware builds, `mise run build-bridge` and `mise run e2e` pass.

## 4. staticcheck on the desktop calibrator

- [ ] 4.1 Add staticcheck with `go get -tool honnef.co/go/tools/cmd/staticcheck@v0.8.2` in `desktop/`, and run `go tool staticcheck ./...` in `mise run desktop-test` between `go vet` and `go test`. Verify `desktop/go.mod` has the `tool` line, `desktop/go.sum` its checksums, and a deliberate unused assignment in a desktop package makes `mise run desktop-test` fail.
- [ ] 4.2 Fix the existing staticcheck findings in `desktop/`. Verify `mise run desktop-test`, `mise run desktop-build`, `mise run e2e` and `mise run vulncheck` pass.

## 5. Gates and documentation

- [ ] 5.1 Add a `lint` job to `.github/workflows/ci.yml` running `mise run lint`, with the same pinned checkout and mise actions as the other jobs. Verify the workflow parses as YAML and every `uses:` is pinned to a commit SHA.
- [ ] 5.2 List `mise run lint` in the root `CLAUDE.md` (commands and verification gates, with how to reformat), name staticcheck in the `desktop-test` line of the root and `desktop/CLAUDE.md`, and add the static checks to `CHANGELOG.md` under Unreleased. Verify `grep` finds `mise run lint` in the gates line and staticcheck in both `CLAUDE.md` files.
- [ ] 5.3 Run every gate from the worktree: `mise run lint`, `mise run test`, `mise run build` (with and without `--serial`), `mise run build-bridge`, `mise run desktop-test`, `mise run desktop-build` and `mise run e2e`. Verify all pass.
