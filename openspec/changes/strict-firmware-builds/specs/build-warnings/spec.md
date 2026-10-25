## Purpose

Compiler warnings in Spooder's own sources fail the build, in the robot firmware, the bridge firmware and the host tests, while warnings from the pinned SDKs do not.

## ADDED Requirements

### Requirement: Warnings in Spooder's own sources fail the build

The robot firmware build, the bridge firmware build and the host test build SHALL treat every compiler warning raised in Spooder's own sources as an error. Spooder's own sources are the files under `core/`, `firmware/servo2040/src/`, `firmware/bridge/main/` and `test/`, and the headers they include from those directories. The robot firmware SHALL compile them with at least `-Wall -Wextra`, and the host tests with at least `-Wall -Wextra`. The bridge firmware SHALL keep the warning set ESP-IDF applies to components and make it fatal for the bridge's own component.

#### Scenario: Warning in shared core code

- **WHEN** a file under `core/` that the robot, the bridge or the host tests compile contains code that raises a compiler warning
- **THEN** that build fails and names the file and the warning

#### Scenario: Warning in a robot adapter

- **WHEN** a file under `firmware/servo2040/src/` raises a compiler warning, in the link build or the `--serial` build
- **THEN** `mise run build` fails

#### Scenario: Warning in a bridge source

- **WHEN** a file under `firmware/bridge/main/` raises a compiler warning
- **THEN** `mise run build-bridge` fails

#### Scenario: Clean tree

- **WHEN** the gates run on the delivered tree
- **THEN** `mise run build` (with and without `--serial`), `mise run build-bridge` and `mise run test` pass with no compiler warning from Spooder's own sources

### Requirement: SDK warnings do not fail the build

Warnings raised inside the pinned SDKs and components (`external/`, the bridge's `managed_components/`, the pico-sdk and pimoroni-pico sources and the Unity framework) SHALL NOT fail any build, including warnings raised in an SDK header that one of Spooder's own sources includes. Spooder's own warnings SHALL NOT be silenced to achieve this: no warning SHALL be disabled for Spooder's own sources, except by a source-local suppression that names the SDK construct it works around.

#### Scenario: Warning in an SDK source

- **WHEN** a pico-sdk, pimoroni-pico, ESP-IDF, Arduino, Bluepad32, BTstack, LovyanGFX or Unity source file raises a warning
- **THEN** the build continues as before

#### Scenario: Warning in an SDK header included by Spooder code

- **WHEN** a Spooder source includes an SDK header that raises a warning under Spooder's flags
- **THEN** the build does not fail on that header's warning, and warnings in the Spooder source itself still fail it
