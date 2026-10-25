# Tasks

## 1. Warnings as errors

- [ ] 1.1 Add `-Wall -Wextra -Werror` as the native env's `build_src_flags` in `platformio.ini` and fix every warning that surfaces in `core/` and `test/` in the code, starting with the `Bar` initialisers in `core/status/status_screen.cpp`. Verify `mise run test` passes; that `pio test -e native -v` shows `-Werror` on `core/` and `test/` compiles and not on Unity's; and that a deliberate unused variable in a `core/` file and in a test file each fails `mise run test` (then removed).
- [ ] 1.2 Add `-Werror` to `spooder_core`'s compile options and to the `spooder` sources' `COMPILE_OPTIONS` property in `firmware/servo2040/CMakeLists.txt`, fixing any warning that surfaces. Verify `mise run build` and `mise run build --serial` pass from a clean `build/servo2040`, that the compile commands show `-Werror` on Spooder's sources and not on pico-sdk or pimoroni-pico sources, and that a deliberate unused variable in `src/main.cpp` and in `core/robot/robot_tick.cpp` each fails the build (then removed).
- [ ] 1.3 Make warnings fatal for the bridge's `main` component only in `firmware/bridge/main/CMakeLists.txt`, with `-Werror` and the `-Werror=` overrides of ESP-IDF's `-Wno-error=` exceptions listed in design.md, fixing each warning in Spooder's sources and handling any from an SDK header at its include as design.md describes. Verify `mise run build-bridge` passes from a clean build, that the compile commands show `-Werror` on `main/` and the shared `core/` files and not on other components, and that a deliberate unused variable in `main/bridge.cpp` fails the build (then removed).

## 2. Robot image budget

- [ ] 2.1 Add `firmware/servo2040/scripts/image_budget.py` (objdump section parser, flash and RAM measurement, the named budgets, the comparison and the use lines) and `test_image_budget.py` covering both budgets fitting, a use exactly at a budget, flash one byte over, RAM over, debug and attribute sections counting towards neither, `.data` counting towards both, and output with no sections. Verify `python -m unittest -v test_image_budget` passes in that directory.
- [ ] 2.2 Add `check_image_budget.py`, run it as the last step of `mise run build`, and add a `test-robot-build` task run by `mise run test`. Verify `mise run build` and `mise run build --serial` print both use lines, each equal to the totals computed by hand from `arm-none-eabi-objdump -h build/servo2040/spooder.elf` under design.md's measurement rule, also on a second, up-to-date build; that temporarily lowering `FLASH_BUDGET` and then `RAM_BUDGET` below today's use fails `mise run build` with the overflow message and makes `mise run upload` fail before looking for a board (then reverted); and that `mise run test` runs the budget test.

## 3. Documentation

- [ ] 3.1 Record the strict warnings and the image budget in `CHANGELOG.md` under Unreleased, and in `firmware/servo2040/CLAUDE.md` (flags, budgets, the check), `firmware/bridge/CLAUDE.md` (warnings fatal in `main/`, SDK headers at the include) and `test/CLAUDE.md` (host flags). Verify `openspec validate strict-firmware-builds` passes and each file names the new gate.
