## Purpose

Build-time guarantees for the robot firmware on the Servo 2040, so that an image growing towards the board's limits fails the build early instead of failing to link or to run.

## ADDED Requirements

### Requirement: The robot image stays within its flash and RAM budgets

After every robot firmware build, `mise run build` SHALL measure the built image's flash use and static RAM use and compare each with a named budget:

- flash use is the number of bytes the image loads into the board's flash: every allocated section with contents whose load address lies in flash, including initialised data and code copied to RAM at boot;
- static RAM use is the number of bytes of SRAM the linker reserves: every allocated section whose run address lies in SRAM, including the reserved minimum heap and stack.

The flash budget SHALL be 256 KiB and the static RAM budget 64 KiB, each below the board's 2 MiB flash and 264 KiB SRAM. The build SHALL fail when either use exceeds its budget, naming the use, the budget and the overflow. A use exactly equal to its budget SHALL pass. When both fit, the build SHALL print one line per budget in the form `flash 142.3 KiB of 256 KiB budget, 44 % free`, where KiB is 1024 bytes, sizes have one decimal and the free share is rounded down. The check SHALL run for the link build and the `--serial` build, on every `mise run build` including when nothing was recompiled, and therefore before every `mise run upload`.

#### Scenario: Image within both budgets

- **WHEN** the built image uses less flash and static RAM than the budgets
- **THEN** the build succeeds and prints both use lines

#### Scenario: Use exactly at a budget

- **WHEN** the flash use equals the flash budget
- **THEN** the build succeeds and reports 0 % free for flash

#### Scenario: Flash budget exceeded by one byte

- **WHEN** the flash use is one byte larger than the flash budget
- **THEN** the build fails with a message giving the flash use, the budget and the one-byte overflow

#### Scenario: RAM budget exceeded

- **WHEN** the static RAM use is larger than the RAM budget while flash fits
- **THEN** the build fails with a message giving the RAM use, the budget and the overflow

#### Scenario: Upload after a failed check

- **WHEN** `mise run upload` builds an image that exceeds a budget
- **THEN** the task fails and nothing is copied to the board

### Requirement: The budget logic is host-tested

The measurement and comparison SHALL be covered by a host-side test that needs no build, toolchain or board, using crafted section tables for at least: an image within both budgets, a use exactly at a budget, flash one byte over, RAM over, sections outside flash and SRAM (debug and attribute sections) that count towards neither, and initialised data that counts towards both. `mise run test` SHALL run it.

#### Scenario: Test run

- **WHEN** `mise run test` runs
- **THEN** the budget test runs and passes alongside the existing host tests
