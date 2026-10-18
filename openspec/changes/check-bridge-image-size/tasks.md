# Tasks

## 1. Checking logic

- [x] 1.1 Add `firmware/bridge/scripts/image_size.py` (binary table parser, `partitions.csv` reader, the four checks and the headroom line) and verify it imports with no SCons or ESP-IDF on the path
- [x] 1.2 Add `firmware/bridge/scripts/test_image_size.py` covering fit, exact fit, one byte too big, missing factory partition, a table differing from the CSV and a wrong app offset, and verify `mise run test-bridge-build` passes
- [x] 1.3 Add the `test-bridge-build` mise task and run it from `mise run test`; verify `mise run test` passes

## 2. Build integration

- [x] 2.1 Add `firmware/bridge/scripts/check_image_size.py` as a `post:` extra script in `platformio.ini`, gating `upload`; verify `mise run build-bridge` passes and prints the headroom line, also on a second, up-to-date build
- [x] 2.2 Temporarily point `board_build.partitions` at the framework's 1 MB `partitions_singleapp.csv`, then separately shrink the factory row in `partitions.csv` to 1 MB, and verify each fails the build with the check's message on two consecutive runs and that `upload-bridge` fails before any upload; revert
- [x] 2.3 Update the partition-table note in `CLAUDE.md` and verify `openspec validate check-bridge-image-size` passes
