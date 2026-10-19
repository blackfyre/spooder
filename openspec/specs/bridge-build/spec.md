# bridge-build Specification

## Purpose
Build-time guarantees for the gamepad bridge firmware, so that an image the board cannot boot fails the build instead of failing on the bench.

## Requirements

### Requirement: The app image fits its app partition

The bridge build SHALL compare the size of the built app image with the size of the factory app partition in the built partition table, the table that is flashed with it. The build SHALL fail, naming both sizes and the overflow, when the image is larger than the partition. An image exactly as large as the partition SHALL pass. When the image fits, the build SHALL print the image size, the partition size and the free share, in the form `app 1.52 MB of 3.88 MB factory partition, 60 % free`, where MB is 1024 × 1024 bytes and the free share is rounded down. The check SHALL run on every bridge build and before every bridge upload, including when nothing was recompiled.

#### Scenario: Image fits

- **WHEN** the built image is smaller than the factory partition
- **THEN** the build succeeds and prints the image size, the partition size and the free share

#### Scenario: Image fills the partition exactly

- **WHEN** the built image is exactly as large as the factory partition
- **THEN** the build succeeds and reports 0 % free

#### Scenario: Image too big by one byte

- **WHEN** the built image is one byte larger than the factory partition
- **THEN** the build fails with a message that the image does not fit, giving both sizes and the one-byte overflow

#### Scenario: Upload after a failed check

- **WHEN** `mise run upload-bridge` builds an image that does not fit
- **THEN** the task fails and nothing is flashed

### Requirement: The built partition table is the project's table

The bridge build SHALL fail when the built partition table has no factory app partition, or when its factory partition's offset or size differs from the factory partition in `firmware/bridge/partitions.csv`. The build SHALL also fail when the app image is flashed at an offset other than the factory partition's.

#### Scenario: No factory partition

- **WHEN** the built partition table has no factory app partition
- **THEN** the build fails with a message saying so

#### Scenario: Table selected from elsewhere

- **WHEN** the built partition table's factory partition is 1 MB while `firmware/bridge/partitions.csv` declares a larger one
- **THEN** the build fails with a message giving both sizes, even if the image would fit the built table

### Requirement: The checking logic is host-tested

The size and table checks SHALL be covered by a host-side test that needs no ESP-IDF or board, using crafted partition tables and image sizes for at least: an image that fits, an exact fit, an image one byte too big, a missing factory partition and a table that differs from the project table. `mise run test` SHALL run it.

#### Scenario: Test run

- **WHEN** `mise run test` runs
- **THEN** the checking-logic test runs and passes alongside the existing host tests
