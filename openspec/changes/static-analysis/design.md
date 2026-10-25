# Design

## Context

Spooder's own C++ is about 115 files across `core/`, `firmware/servo2040/src/`, `firmware/bridge/main/` (including one C file, `main.c`), `test/`, `tools/virtual-robot/` and `tools/bridge-screenshots/`. The style is consistent by hand: Allman braces, two-space indent, namespaces unindented, single-statement `if` bodies without braces, and lines mostly under 120 columns (a handful reach 130–160). There is no `.clang-format` or `.clang-tidy` yet, and no clang tool on the developer machine or in `mise.toml`.

The host tests build `core/` through PlatformIO's `native` environment (`platformio.ini`, `-std=gnu++17 -Wall -Icore`, the core sources listed in `build_src_filter`). The desktop calibrator's gate, `mise run desktop-test`, runs `gofmt`, `go vet` and `go test`; `mise run vulncheck` already pins a Go tool by version. See proposal.md for the scope and the spec for the requirements.

## Goals / Non-Goals

**Goals:**
- A formatting check that fails on drift, with as little reformatting as possible now.
- clang-tidy findings that are worth fixing, on the code that runs on both the robot and the host.
- staticcheck on the calibrator in the gate developers already run.
- Every tool pinned, and every check in `CLAUDE.md`'s gates and CI.

**Non-Goals:**
- Changing the house style, or rewriting code to please a check.
- Analysing the cross-compiled firmware adapters or the ESP-IDF bridge.

## Decisions

### All three parts in one change
The brief has three independent parts (clang-format, clang-tidy, staticcheck). They are delivered together because they share the wiring: one pinned-tool block in `mise.toml`, one CI change, one entry in the gates, one changelog entry. Each part is small, and their tasks are grouped so that each one can be checked on its own.

### clang-format and clang-tidy 22.1.8 from PyPI, through mise
mise's `pipx` backend already installs PlatformIO through `uv`; the same backend installs the `clang-format` and `clang-tidy` wheels (`"pipx:clang-format" = "22.1.8"`, `"pipx:clang-tidy" = "22.1.8"`). The wheels are self-contained binaries, so CI and developer machines get the same tool without a distribution package.
- **Why 22.1.8:** both wheels exist at that version (released 2026-07-11 and 2026-07-14), well past the project's seven-day rule, and the two tools share one LLVM release. 23.1.3 of clang-tidy is only seven days old.
- **Checked:** the 22.1.8 clang-tidy wheel finds the C++ standard headers through the host compiler and reports `bugprone-narrowing-conversions` on a test file.
- **Alternatives:** distribution packages (unpinned, different per machine); a mise LLVM plugin (no maintained one with exact pins).

### The formatting style is derived from the code
The `.clang-format` starts from `BasedOnStyle: LLVM` with `BreakBeforeBraces: Allman`, `IndentWidth: 2` and `NamespaceIndentation: None`. Then the column limit, `AllowShortIfStatementsOnASingleLine`, include sorting, `PointerAlignment`/`ReferenceAlignment` (the code writes `ResumeGate &g`), comment reflow and argument packing are set to whichever value changes the fewest lines, measured by running clang-format over the file set with each candidate. `SortIncludes` stays off unless it changes nothing, since include order can matter in the firmware. The chosen values and the resulting changed-line count are recorded in this file during implementation.

**Measured (task 2.1)** with clang-format 22.1.8 over the 114 checked files (the 120 the lint lists, less the 6 under the exclusions), counting changed lines as lines removed plus lines added by a line diff of each file against its clang-format output. The base (LLVM, Allman, indent 2, unindented namespaces, includes not sorted) changes 3537 lines. A coordinate search over 49 options, each tried at its usual values in turn, repeated until no option improved, settled on:

| Option | Chosen | Lines at the chosen value, and at the others |
|---|---|---|
| `ColumnLimit` | 120 | 979; 80: 3296, 100: 1425, 110: 1097, 130: 1059, 140: 1124, 160: 1155, 0 (none): 1744 |
| `AllowShortIfStatementsOnASingleLine` | `Never` | 979; `WithoutElse` 1893, `OnlyFirstIf` 1962, `AllIfsAndElse` 2103 |
| `AllowShortFunctionsOnASingleLine` | `Inline` | 979; `All` (LLVM) 1244, `InlineOnly` 1163, `Empty` 1029, `None` 1229 |
| `PointerAlignment` / `ReferenceAlignment` | `Right` / `Pointer` | 979; `Left` 2279, `Middle` 2274 / `Left` 1879, `Middle` 1878 |
| `ReflowComments` | `Never` | 1010 against 1022 for `Always` (LLVM) before the last options settled, a tie at 979 after; kept so hand-wrapped comments stay as written |
| `BinPackArguments` / `BinPackParameters` | `true` / `BinPack` (LLVM) | 979; `false` 1251 / `OnePerLine` 1103 |
| `IndentExternBlock` | `NoIndent` | 979; `AfterExternBlock` (LLVM with Allman) and `Indent` 987: they indent the body of `extern "C" {` in the bridge's C header |
| `MaxEmptyLinesToKeep` | 2 | 979; 1 (LLVM) 980 |
| `PenaltyBreakAssignment` | 100 | 979; 2 (LLVM) 1001, 20 989 |
| `SortIncludes` | `Never` | sorting (`CaseSensitive`) changes 6 lines, so it stays off |

Every other option searched (alignment of assignments, declarations, macros, operators and trailing comments, short loops, blocks, case labels, lambdas and enums, constructor initialisers, binary and ternary operator breaks, template and return-type breaks, continuation and initialiser indents, case-label and preprocessor indents, access modifiers, braced lists, spaces before parentheses, after casts and before trailing comments, empty lines, namespace comments, definition blocks, the final newline) changed the fewest lines at its LLVM value, so `.clang-format` does not list it.

One value was not taken from the count: `PenaltyExcessCharacter: 10` changes 12 fewer lines (967), but only by letting clang-format write lines past the 120-column limit (a 124-column `if` in the virtual robot, for one), which makes the limit meaningless. It keeps the LLVM value, so the limit holds.

With the four data tables fenced by `// clang-format off`/`on` (the logo bitmap in `firmware/bridge/main/logo.h`, `POWER_LIMITS` in `firmware/servo2040/src/config.h`, and the target and pose tables of the leg-pose and centre-of-gravity golden tests, which clang-format would pack into columns), the reformat changes **803 lines** across 61 files. The largest are `core/movement/vectors.h` (one-line accessors written as blocks), `core/protocol/protocol.h` (trailing-comment alignment) and `firmware/bridge/main/main.c` (the Bluepad32 template's four-space indent and attached braces).

### Exclusions through `.clang-format-ignore`
clang-format (18 and later) honours a `.clang-format-ignore` at the root, so the exclusions hold in editors as well as in the lint task:
- `external/`, `managed_components/`: third-party code;
- `firmware/bridge/main/fonts/`: generated by `make_fonts.py`;
- `test/golden/`: fixtures other implementations compare against byte for byte;
- `core/movement/calibration_defaults.cpp`: written by the desktop calibrator's export in a fixed format, pinned by `test/golden/calibration_defaults.cpp`. Reformatting it would make every export fail the lint, or the export diverge from its fixture.

Large data tables in hand-written headers (`firmware/bridge/main/logo.h`, the portal page) are kept as they are with `// clang-format off` / `on` if formatting them would churn them.

### The lint task
`mise run lint` runs a script in `tools/` (like `vulncheck.sh`), which:
1. lists the C++ files in the checked directories (`*.c`, `*.cpp`, `*.h`, `*.hpp`) with `git ls-files --cached --others --exclude-standard`, so a new file counts before it is staged and git-ignored build output never does;
2. runs `clang-format --dry-run --Werror` on them, which reports each unformatted file and modifies nothing;
3. removes any old `compile_commands.json` at the project root, writes the host compile database with `pio run -e native -t compiledb` (PlatformIO writes it at the project root by default; same flags as the host tests), checks that it exists and lists every source in the `native` environment's `build_src_filter`, and runs clang-tidy on each `core/` source in it. `compile_commands.json` is added to `.gitignore`.

It fails if either step fails. A developer fixes formatting with `clang-format -i` on the files named (documented in the root `CLAUDE.md`), not through the lint task.

### The clang-tidy check set
`.clang-tidy` at the root:
- `bugprone-*`, without `bugprone-easily-swappable-parameters` (it fires on most functions taking two floats and asks for a rewrite, not a fix);
- `bugprone-narrowing-conversions` stays on (in clang-tidy 22 it is the primary name, `cppcoreguidelines-narrowing-conversions` its alias), with `WarnOnFloatingPointNarrowingConversion` kept;
- `performance-*` without `performance-avoid-endl` (no streams in `core/`);
- `readability-misleading-indentation`, `readability-duplicate-include`, `readability-inconsistent-declaration-parameter-name`, `readability-redundant-declaration`, `readability-non-const-parameter`, `readability-container-size-empty`;
- `WarningsAsErrors: '*'`, `HeaderFilterRegex` matching `core/`, so findings in `core/` headers are reported and those in `external/` and system headers never. A header included by several sources may report the same finding more than once; the gate only needs a non-zero exit, so no deduplication is done.

A check that turns out to be mostly noise on `core/` is disabled in `.clang-tidy` with a comment giving the reason, rather than suppressing it line by line. A real but intentional finding (a deliberate narrowing in fixed-point protocol encoding, for example) gets `NOLINTNEXTLINE(check)` and a reason on the same comment.

### staticcheck through a `tool` directive
`go get -tool honnef.co/go/tools/cmd/staticcheck@v0.8.2` (staticcheck 2026.2.2, released 2026-10-12, supporting Go 1.27) adds the `tool` line to `desktop/go.mod` and its checksums to `desktop/go.sum`, so Dependabot proposes its updates with the other Go modules. `desktop-test` runs `go tool staticcheck ./...` between `go vet` and `go test`, with the default check set; a check is disabled only in a `desktop/staticcheck.conf` with a comment giving the reason.

### CI
A `lint` job on `ubuntu-24.04` with the same pinned checkout and mise actions as the others runs `mise run lint`. It needs no submodules: `core/` includes nothing from `external/`. staticcheck needs no change in CI, because the `desktop` job already runs `mise run desktop-test`.

## Assumptions

- The formatting check covers untracked files that git does not ignore, so a developer sees drift before staging; CI sees only committed files, which is the same set on a clean checkout.
- The brief names `firmware/bridge/src`; it does not exist, so `firmware/bridge/main` is the bridge's only source directory.
- `core/movement/calibration_defaults.cpp` counts as generated (the desktop export writes it) and is excluded from formatting, even though the brief does not name it.
- The host compile database for clang-tidy is PlatformIO's `native` environment, the one the host tests use; it lists exactly the core sources in `build_src_filter`. A `core/` source missing from that list is not analysed; adding it is a matter for the change that adds the source.
- The narrowing-conversion checks are `bugprone-narrowing-conversions` only; the `cppcoreguidelines-*` alias is not enabled as well, to avoid double reports.
- `tools/vulncheck.sh` scans the packages the calibrator imports, so staticcheck's modules in `go.sum` do not change its baseline; if they do, the baseline task is answered in the staticcheck task.
- `tools/lint.sh` fails closed: without a root `.clang-format` (or `.clang-tidy`) the formatting check (or clang-tidy) fails rather than falling back to a default style or skipping, so the task fails between task 1.1 and tasks 2.1 and 3.1, which add the configurations. It also fails when the `clang-format` or `clang-tidy` on `PATH` is not the version pinned in `mise.toml`, so running the script outside `mise run lint` cannot use another LLVM.
- `.git-blame-ignore-revs` names the reformat commit by its hash on `deliver/static-analysis`. The merge to `main` must keep that commit (a fast-forward or merge, not a squash); if a rebase rewrites it, the hash in the file is updated in the same rebase.
- Until task 3.1 adds `.clang-tidy`, `mise run lint` exits non-zero by design (the clang-tidy half fails closed), so task 2.2's checks read the formatting half of its output: the files it names, or none.
- clang-tidy's findings in `core/` will be few enough to fix or suppress by hand within this change. If a single check yields many findings of one harmless pattern, it is disabled with a reason rather than rewritten across the code.

## Risks / Trade-offs

- **Reformatting hides history** → it lands in one commit of its own, whose hash goes into `.git-blame-ignore-revs`.
- **A fix for a finding changes behaviour** (a narrowing turned into an explicit cast is safe; changing a type is not) → fixes keep types and arithmetic as they are and make a conversion explicit only where the value's range is shown to fit the target type (clang-tidy stops warning at an explicit cast, so the cast alone proves nothing); in the protection maths and the protocol encoding the bound is named in a comment beside the cast, and a conversion whose range is not bounded is reported as a finding, not cast; the host tests, the golden-vector tests and the end-to-end tests must pass unchanged.
- **The PyPI wheels are third-party builds of LLVM** → they are pinned exactly and come from the long-standing `ssciwr` packaging; a later change can move to an official LLVM release archive if mise gains a pinned backend for it.
- **clang-tidy of `core/` sees only host flags** → code under `#ifdef` for the RP2040 or ESP32 is not analysed; that is a follow-up below.

## Follow-ups

- clang-tidy on `firmware/servo2040/src/` with the pico-sdk CMake compile database (`CMAKE_EXPORT_COMPILE_COMMANDS`), and on the virtual robot's CMake build.
- clang-tidy on `firmware/bridge/main/` with the ESP-IDF compile database.
- clang-tidy on `test/` and `tools/bridge-screenshots/`.
- A pre-commit hook that runs the formatting check on staged files.
