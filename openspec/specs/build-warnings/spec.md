# build-warnings Specification

## Purpose
Compiler warnings in Spooder's own sources fail the build, in the robot firmware, the bridge firmware and the host tests, while warnings from the pinned SDKs do not.

## Requirements

### Requirement: Warnings in Spooder's own sources fail the build

The robot firmware build, the bridge firmware build and the host test build SHALL treat every compiler warning raised in Spooder's own sources as an error. Spooder's own sources are the files under `core/`, `firmware/servo2040/src/`, `firmware/bridge/main/` and `test/`, and the headers they include from those directories. The robot firmware, the bridge firmware and the host tests SHALL each compile them with at least `-Wall -Wextra`, and no warning that `-Wall` or `-Wextra` enables SHALL be switched off for them, including by an SDK's default flags.

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

Warnings raised inside the pinned SDKs and components (`external/`, the bridge's `managed_components/`, the pico-sdk and pimoroni-pico sources and the Unity framework) SHALL NOT fail any build. A warning that a header of the pinned SDK versions raises when one of Spooder's own sources includes it SHALL NOT fail the build either: it is suppressed at that include, for that header only, and an SDK pin change that makes an included header raise a new warning SHALL carry that suppression in the same change. The SDK include directories are not marked as system directories, so a header warning is handled where it arises rather than hidden for every SDK. Spooder's own warnings SHALL NOT be silenced to achieve this: no warning SHALL be disabled for Spooder's own sources, except by a source-local suppression that names the SDK construct it works around.

#### Scenario: Warning in an SDK source

- **WHEN** a pico-sdk, pimoroni-pico, ESP-IDF, Arduino, Bluepad32, BTstack, LovyanGFX or Unity source file raises a warning
- **THEN** the build continues as before

#### Scenario: Warning in an SDK header included by Spooder code

- **WHEN** a Spooder source includes a header of a pinned SDK that raises a warning under Spooder's flags
- **THEN** that warning is suppressed at the include, for that header only, the build does not fail on it, and warnings in the Spooder source itself still fail it
