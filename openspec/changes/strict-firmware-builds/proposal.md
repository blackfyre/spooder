# Proposal

## Why

Warnings in Spooder's own code pass every gate today: the robot firmware compiles its sources with `-Wall` and `-Wextra` but builds on, the host tests compile `core/` with `-Wall` only, and the bridge firmware takes whatever ESP-IDF sets, which is neither `-Wall` nor `-Wextra`. Turning on `-Wextra` for the host tests already shows missing field initialisers in `core/status/status_screen.cpp`, code that the bridge ships. A warning that nobody has to read is a bug report nobody receives.

The robot image has no size gate either. The bridge build fails when its app no longer fits its partition (`bridge-build`), but the robot build would only notice at link time, when flash or RAM is already full, and gives no early sign of creeping use. Today the robot image loads 145,724 bytes (142.3 KiB) into its 2 MiB flash and reserves 42,512 bytes (41.5 KiB) of its 264 KiB SRAM statically, measured at `4424940` on the link build.

## What Changes

- Warnings are errors in Spooder's own sources, never the SDKs':
  - the robot firmware: `-Werror` on the same scopes that carry `-Wall`/`-Wextra` today (the `spooder` sources and `spooder_core`);
  - the host tests: `-Wall -Wextra -Werror` on the native environment;
  - the bridge firmware: `-Wall -Wextra -Werror` on its `main/` component, which holds the bridge sources and the shared `core/` files it compiles, not `managed_components/` or `external/`, with the warnings ESP-IDF switches off by default switched back on and its `-Wno-error=` exceptions made fatal.
- Every warning that surfaces is fixed in the code, not suppressed; a warning raised inside an SDK header is kept out of scope by including that header as a system header.
- `mise run build` checks the robot image against named flash and static-RAM budgets after every build, both the link build and `--serial`, and fails when either is exceeded. When it fits it prints the use, the budget and the free share.
- A host test covers the budget check with crafted section tables; `mise run test` runs it.
- `CHANGELOG.md` and the robot, bridge and test `CLAUDE.md` files record the new gates.

## Capabilities

### New Capabilities

- `build-warnings`: Spooder's own sources build without warnings in the robot firmware, the bridge firmware and the host tests.
- `robot-build`: build-time guarantees for the robot firmware, starting with "the image stays within its flash and RAM budgets".

### Modified Capabilities

None.

## Impact

- `firmware/servo2040/CMakeLists.txt`, `platformio.ini`, `firmware/bridge/main/CMakeLists.txt`.
- Source fixes wherever a warning surfaces (known so far: `core/status/status_screen.cpp`).
- A new `firmware/servo2040/scripts/` directory with the budget check and its test; `mise.toml` (`build` runs the check, a `test-robot-build` task runs its test from `mise run test`).
- `CHANGELOG.md`, `firmware/servo2040/CLAUDE.md`, `firmware/bridge/CLAUDE.md`, `test/CLAUDE.md`.
- No runtime behaviour or link protocol changes; nothing needs flashing.
