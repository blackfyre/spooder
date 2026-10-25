# Design

## Context

See proposal.md for the motivation and specs/ for the requirements.

- **Robot.** `firmware/servo2040/CMakeLists.txt` gives `spooder_core` `-Wall -Wextra` (PRIVATE), and the `spooder` target `-Wall` with `-Wextra` set as a source-file property on its own sources only, because the SDK's INTERFACE libraries compile their sources into the same target. A scratch build of both variants at `4424940` with today's flags shows no warning.
- **Host tests.** `platformio.ini`'s native env has `build_flags = -std=gnu++17 -Wall -Icore`. With `-Wextra -Werror` added, `core/status/status_screen.cpp` fails on `missing-field-initializers` for `status_screen::Bar::heldFraction` and `held` (lines 212 and 214); the run stopped at the first failing test, so more may follow.
- **Bridge.** `firmware/bridge/main/CMakeLists.txt` registers the bridge sources and the shared `core/` files it uses as one ESP-IDF component; Arduino, Bluepad32, BTstack and LovyanGFX are other components, included with plain `-I`.
- **Robot image today** (`arm-none-eabi-objdump -h` of `build/servo2040/spooder.elf`, Release, `power_limits.example.h`, at `4424940`):

  | Build | Flash use | Static RAM use |
  |---|---|---|
  | link (default) | 145,724 B (142.3 KiB) | 42,512 B (41.5 KiB) |
  | `--serial` | 145,592 B (142.2 KiB) | 41,324 B (40.4 KiB) |

  Flash: `.boot2` 256, `.text` 123,636, `.rodata` 11,008, `.ARM.extab` 396, `.ARM.exidx` 664, `.binary_info` 32 and `.data` 9,732 (loaded from flash, run from RAM). RAM: `.ram_vector_table` 192, `.data` 9,732, `.bss` 28,492, `.heap` 2,048 (the minimum reserve) and `.stack_dummy` 2,048 (core 0's stack in scratch Y). The board has 2 MiB of flash (`PICO_FLASH_SIZE_BYTES`) and 264 KiB of SRAM.

## Goals / Non-Goals

**Goals:**

- Make warnings fatal exactly where Spooder already asks for them, so the flags do not reach the SDKs' own sources.
- A size gate that runs on every robot build and in CI, with its logic tested on the host like the bridge's.

**Non-Goals:**

- New warning flags beyond `-Wall -Wextra` (`-Wconversion`, `-Wshadow` and the like); they can follow once the tree is clean at this level.
- Warnings in the virtual robot, the bridge screenshot renderer and the desktop calibrator (Go has `go vet` in `desktop-test`). The virtual robot and the screenshot renderer compile `core/` too, so a `core/` warning is already caught by the host tests.
- A RAM figure for the heap actually used at run time, and stack depth; the gate measures what the linker reserves.
- A bridge size budget beyond its existing partition check.

## Decisions

### Robot: `-Werror` on the two existing scopes

`spooder_core` gets `-Werror` next to its `-Wall -Wextra`; the `spooder` target's own sources get `-Werror` in the same source-file `COMPILE_OPTIONS` property that carries `-Wextra`, and the target-wide `-Wall` stays as it is. Putting `-Werror` on the target would also apply it to the SDK sources compiled into it, which the brief and the spec exclude. Because `SPOODER_SOURCES` is read after the `--serial` branch adds its sources, both variants are covered.

### Host tests: `build_src_flags`

`-Wall -Wextra -Werror` go into the native env's `build_src_flags`; `-std=gnu++17` and `-Icore` stay in `build_flags`. In PlatformIO 6.2.0 (`builder/tools/piobuild.py`, `ProcessProjectDeps`) the test sources and, with `test_build_src`, the `core/` sources are both built by the project builder, which takes `build_src_flags`; Unity is built by its own library builder, which does not. `build_flags` would also reach Unity and make any future Unity warning fatal, against the spec.

### Bridge: fatal warnings on the `main` component only

ESP-IDF does not enable `-Wall` or `-Wextra` for components: in the packaged framework's `tools/cmake/build.cmake` (`__build_set_default_build_specifications`) `-Wall`, `-Werror` and `-Wextra` are commented out, and the bridge's compile commands at `4424940` carry only `-Wno-frame-address -Wno-error=unused-function -Wno-error=unused-variable -Wno-error=unused-but-set-variable -Wno-error=deprecated-declarations -Wno-error=extra -Wno-unused-parameter -Wno-sign-compare -Wno-enum-conversion`. The bridge's component therefore asks for the same set as the robot itself: after `idf_component_register`, `firmware/bridge/main/CMakeLists.txt` adds `target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror …)`, followed by the positive form of every warning ESP-IDF switches off that `-Wall`/`-Wextra` would enable (`-Wframe-address -Wunused-parameter -Wsign-compare`, and `-Wenum-conversion` for C sources only through `$<$<COMPILE_LANGUAGE:C>:…>`, since it is a C-only option), and `-Werror=` for each of ESP-IDF's `-Wno-error=` exceptions (`unused-function`, `unused-variable`, `unused-but-set-variable`, `deprecated-declarations`, `extra`). The explicit forms are needed because GCC gives a specific `-W<name>`/`-Wno-<name>` priority over `-Wall`/`-Wextra` whatever their order, while between options of the same specificity the last wins; the implementation confirms from the compile commands that the component's options come after ESP-IDF's. PRIVATE options reach only the component's own sources, the bridge's and the shared `core/` files, never another component. A warning that comes from an Arduino, Bluepad32, BTstack or LovyanGFX header is handled at the include: a Spooder-owned wrapper header with `#pragma GCC system_header`, or a `#pragma GCC diagnostic` push/ignore/pop around that include naming the header, not a project-wide `-Wno-…`.

### Fix warnings in the code

Each warning is fixed where it arises: for example the `Bar` aggregate initialisations in `status_screen.cpp` get their remaining members (or the struct gets default member initialisers), unused parameters lose their names or are used. No `-Wno-…` for Spooder's own sources, which is also why ESP-IDF's `-Wno-…` defaults are reversed for the bridge's component.

### Budget check: a plain Python module run by `mise run build`

Following the bridge's `firmware/bridge/scripts/` layout:

- `firmware/servo2040/scripts/image_budget.py` holds the parsing, the measurement and the comparison, with the budgets as named constants `FLASH_BUDGET = 256 * 1024` and `RAM_BUDGET = 64 * 1024`, and the board's flash and SRAM address ranges.
- `check_image_budget.py <elf>` runs `arm-none-eabi-objdump -h` (the toolchain mise already provides for the build) on the ELF and exits non-zero with the module's message on a failure.
- `test_image_budget.py` (standard `unittest`) feeds crafted `objdump -h` text to the module.
- `mise run build` gains a third step running the check on `build/servo2040/spooder.elf`. Being a mise step, it runs after every build, also an up-to-date one, and `mise run upload` (which calls `mise run build` first) never copies an image that failed it. CI runs `mise run build` with and without `--serial`, so it covers both variants.
- A `test-robot-build` mise task runs the test from `firmware/servo2040/scripts`; `mise run test` runs it after `test-bridge-build`.

Alternatives: a CMake `POST_BUILD` command only runs when the ELF is relinked, so an up-to-date build would print nothing and skip the gate; the linker's own overflow errors only fire at the hardware limit. `arm-none-eabi-size` gives only run addresses, which would count `.data` once instead of in both flash and RAM; `objdump -h` gives both the run (VMA) and load (LMA) address and the section flags. Parsing the ELF in Python would avoid the tool, but the toolchain is always present where the ELF is built.

### Measurement rule

Flash use is the sum of sections flagged `ALLOC` and `LOAD` whose LMA is in `0x10000000`–`0x101FFFFF`; static RAM use is the sum of sections flagged `ALLOC` whose VMA is in `0x20000000`–`0x20041FFF`. This counts `.data` in both, and the minimum heap and the stack reservation in RAM, and ignores `.debug_*`, `.ARM.attributes` and `.comment`. Matching by flags and addresses rather than section names keeps time-critical code copied to RAM and future sections counted without a list to maintain.

### Budget values

Flash 256 KiB: 142.3 KiB used today, 44 % free, 1/8 of the board's flash. Static RAM 64 KiB: 41.5 KiB used today, 35 % free, leaving about 200 KiB for the heap and core 1. Both leave room for the suspended body-sensing work while catching a doubling, such as a large table or buffer added by mistake. Raising a budget is a reviewed one-line change to a named constant, recorded in the changelog.

## Risks / Trade-offs

- [More warnings surface in bridge code than the host run showed] → each is fixed in place; the bridge build is a gate, so none can be skipped.
- [An SDK update adds a warning in a header Spooder includes] → the build fails in Spooder's source; the fix is the include-level handling above, in the same reviewed commit as the pin change.
- [A compiler update adds warnings to `-Wall`/`-Wextra`] → the Arm toolchain is pinned in `mise.toml` and ESP-IDF by the bridge's platform release, so for the firmwares this only happens in a reviewed bump. The host tests use the system's native compiler, which is not pinned: a runner or distribution update can surface a new warning in `mise run test` without any change in the repository. That failure points at real code to fix; pinning a native compiler is out of scope.
- [The objdump output format changes with a toolchain bump] → the parser fails loudly when it finds no sections, rather than reporting zero use.

## Assumptions

Made without a human to ask; each can be revisited in review.

- The bridge's component compiles with `-Wall -Wextra` like the robot, ESP-IDF's own disabled warnings re-enabled, rather than "ESP-IDF's warning set, made fatal": ESP-IDF enables neither `-Wall` nor `-Wextra` for components, so its set alone would leave format, sign and unused-parameter warnings unchecked in the bridge's sources and in the shared `core/` files it compiles.
- Both parts of the brief, warnings as errors and the image budget, are delivered in this one change rather than splitting one off as a follow-up. They are small, share the theme and gates (`mise run build`, `mise run test`), and the brief names them as one change with one changelog entry.
- Budgets are 256 KiB of flash and 64 KiB of static RAM, about 1.8 times and 1.5 times today's use.
- Static RAM counts the linker's minimum heap and the core 0 stack reservation, since they are SRAM the image claims; the 264 KiB SRAM limit is not checked separately because the linker already fails on it and the budget is far below.
- One budget pair serves the link and `--serial` builds; they differ by about 1 KiB.
- The brief's "native env: -Wall -Wextra -Werror" is met with `build_src_flags` in that env rather than `build_flags`, so that Unity stays outside the gate as the spec requires.
- The bridge's C sources also get `-Wold-style-declaration`: ESP-IDF adds `-Wno-old-style-declaration` for C after its common options, and `-Wextra` enables that warning in C, so leaving it off would switch off a `-Wextra` warning against the spec. The bridge's compile commands show the component's options after all of ESP-IDF's, including that one, so the positive forms win.
- LovyanGFX's panel headers raise `-Wunused-parameter` under the bridge's flags (`Panel_AMOLED.hpp`, `Panel_FlexibleFrameBuffer.hpp`, `Panel_FrameBufferBase.hpp`, `Panel_RA8875.hpp`), and it is included from two places (`display.cpp`, `screen_draw.h`), so the include-level handling is one Spooder-owned wrapper, `firmware/bridge/main/lovyangfx_include.h`, with a `#pragma GCC diagnostic` push/ignore/pop of that one warning around `<LovyanGFX.hpp>`, rather than `#pragma GCC system_header` (which would only mark the wrapper itself) or the same pragmas at each include. No other SDK header warns today.
- In the host tests, PlatformIO's generated `unity_config.c` (written into the build directory for the project builder) is compiled with `build_src_flags` like the tests; it is PlatformIO's two-line glue, not Unity's framework source, which stays without `-Werror`.
- The robot image figures above are a reference measurement with `power_limits.example.h` and the MG996R profile before any warning fix; the delivered check is verified against the `objdump -h` of the ELF it measures, not against these numbers.

## Follow-ups

- Further warning flags (`-Wshadow`, `-Wconversion`, `-Wdouble-promotion`) once the tree is clean at `-Wall -Wextra`.
- `-Werror` in the virtual robot and the bridge screenshot renderer, which compile `core/` on the host.
