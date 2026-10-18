# Proposal

## Why

The bridge app image (about 1.6 million bytes) once exceeded its 1 MB factory partition without any warning: the build succeeded, the flash verified, and the board then refused to boot ("Image length … doesn't fit in partition length …"). The platform's own size check does not catch this, because it only takes its limit from an `ota_0` partition and otherwise falls back to the board's full 4 MB flash size. A too-small or wrongly selected partition table must fail the build, not the boot.

## What Changes

- `mise run build-bridge`, and therefore `mise run upload-bridge`, check the built app image against the app partition in the built partition table, and fail with a clear message when it does not fit. Upload does not start after a failed check.
- When the image fits, the build prints the headroom, for example `app 1.52 MB of 3.88 MB factory partition, 60 % free`.
- The build also fails when the built partition table's factory partition differs from the one in `firmware/bridge/partitions.csv`, which catches a table selected from somewhere other than the project file (the sdkconfig-versus-`board_build.partitions` class of bug).
- A host-side test covers the checking logic with crafted partition tables and image sizes.

## Capabilities

### New Capabilities

- `bridge-build`: build-time guarantees for the bridge firmware, starting with "the app image fits the partition it is flashed into".

### Modified Capabilities

None.

## Impact

- `firmware/bridge/platformio.ini` (an `extra_scripts` entry) and a new `firmware/bridge/scripts/` directory holding the check, its PlatformIO hook and its test.
- `mise.toml`: a task that runs the test, also run by `mise run test`.
- `CLAUDE.md`: the partition-table note.
- No firmware source or runtime behaviour changes.
