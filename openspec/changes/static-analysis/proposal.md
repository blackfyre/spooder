# Proposal

## Why

Spooder's C++ and Go are checked only by compilers, `gofmt`, `go vet` and tests. Nothing keeps the C++ layout consistent across the two firmwares, the host tools and the tests, and no static analyser looks for the bug classes that compile cleanly: narrowing conversions in the kinematics and protection maths, swapped arguments, misleading indentation, dead stores, and the Go mistakes `go vet` does not catch. Reviews spend attention on these instead of on behaviour.

## What Changes

- **C++ formatting:** a `.clang-format` at the repository root, derived from the existing style (Allman braces, two-space indent) to keep churn small, applied to Spooder's own C++: `core/`, `firmware/servo2040/src/`, `firmware/bridge/main/`, `test/`, `tools/virtual-robot/` and `tools/bridge-screenshots/`. Never `external/`, `managed_components/`, the generated fonts in `firmware/bridge/main/fonts/`, `test/golden/`, or `core/movement/calibration_defaults.cpp`, whose fixed format the desktop calibrator writes. Any reformatting lands in a commit of its own.
- **C++ static analysis:** clang-tidy on `core/` with a small, high-signal check set (`bugprone-*` with narrowing conversions, a few `performance-*` and `readability-*` checks), run against the host compile database. Existing findings are fixed, or suppressed in place with a reason; no mass style rewrites.
- **One lint task:** `mise run lint` checks the formatting and runs clang-tidy, and fails on any unformatted file or finding.
- **Go static analysis:** staticcheck, pinned through a `tool` directive in `desktop/go.mod` (so `desktop/go.sum` pins it), runs in `mise run desktop-test`. Existing findings are fixed.
- **Pinned tools:** clang-format and clang-tidy 22.1.8, pinned in `mise.toml`.
- **Gates:** `mise run lint` joins the verification gates in the root `CLAUDE.md` and gets its own CI job; staticcheck runs in CI through `desktop-test`.

**Non-goals:**
- clang-tidy on the firmware adapters, the bridge, the virtual robot, the tests or the host tools (see the follow-ups in design.md);
- Python, shell, YAML or Markdown linting;
- any change of behaviour.

## Capabilities

### New Capabilities

- `static-checks`: the formatting check and static analysis of Spooder's own C++ and Go: what they cover and exclude, how the tools are pinned, and how they fail the gates.

### Modified Capabilities

None. No firmware, link or calibrator behaviour changes.

## Impact

- **New:** `.clang-format`, `.clang-format-ignore`, `.clang-tidy`, the lint script behind `mise run lint`.
- **Changed:**
  - `mise.toml`: the pinned clang tools, the `lint` task, staticcheck in `desktop-test`;
  - `desktop/go.mod`, `desktop/go.sum`: the staticcheck tool directive;
  - C++ sources in the formatted directories (layout only, in its own commit), and `core/` and `desktop/` sources where a finding is fixed;
  - `.github/workflows/ci.yml`: a lint job;
  - `CLAUDE.md`, `desktop/CLAUDE.md`, `CHANGELOG.md`.
- **Dependencies:** the `clang-format` and `clang-tidy` 22.1.8 wheels from PyPI (through mise's pipx backend and `uv`), and staticcheck 2026.2.2 (`honnef.co/go/tools` v0.8.2).
- **Risk:** low. Formatting and fixes for findings must not change behaviour; the existing gates (host tests, both firmware builds, the desktop tests and the end-to-end tests) confirm it. No flashing is needed beyond the usual check that the firmwares still build.
