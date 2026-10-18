# Design

## Context

The pioarduino platform builds `partitions.bin` from `board_build.partitions` (it ignores the sdkconfig partition setting) and runs PlatformIO's `checkprogsize` before producing `firmware.bin`. That check takes its limit from `upload.maximum_size`, which the platform only lowers to an `ota_0` partition's size. Our table has a factory partition and no OTA slots, so the limit stays at the `esp32dev` board default of 4 MB, and an image of any size up to the flash size passes. `upload` is a single `pio run -t upload` invocation that builds and flashes in one SCons run.

## Goals / Non-Goals

**Goals:**

- Check the files that are actually flashed: the built `firmware.bin` against the built `partitions.bin`, at the offset the upload writes the app to.
- Run on every build and before every upload, also when SCons finds everything up to date.

**Non-Goals:**

- OTA layouts. The bridge has a single factory app; a table without one fails the check, and supporting OTA slots is a later change if the bridge ever gets OTA.
- Fixing the platform's own `checkprogsize` limit.
- Checking the sdkconfig partition settings; the platform does not use them, and the built table is what matters.

## Decisions

### A PlatformIO post script, not a step in the mise task

`firmware/bridge/scripts/check_image_size.py` is a `post:` entry in `extra_scripts`. It defines an always-built `checkimagesize` alias whose sources are `$BUILD_DIR/${PROGNAME}.bin` and `$BUILD_DIR/partitions.bin`, adds it to the default targets, and makes `upload` depend on it.

- A check run by the mise task after `pio run` cannot gate `upload-bridge`: `pio run -t upload` builds and flashes in one go, so the check would run after the board was written. Splitting the upload task into build, check and upload would work for the mise task but not for `pio run -t upload` run directly or from an IDE.
- Inside SCons the script reads the exact paths and the app offset (`$ESP32_APP_OFFSET`) the platform uses, instead of re-deriving them.
- An always-built alias rather than `AddPostAction` on `firmware.bin`: a post-action only runs when the image is rebuilt, so an up-to-date build or a repeated upload would skip the check and the headroom line. `checkprogsize` cannot carry it either, because `firmware.bin` depends on `checkprogsize`, so it runs before the image exists.

### The checking logic is a plain Python module

`firmware/bridge/scripts/image_size.py` holds the parsing and the checks with no SCons or ESP-IDF imports; the post script only gathers paths and turns a failure into a build error. This lets `test_image_size.py` (standard `unittest`) exercise it on crafted tables and sizes. `mise run test-bridge-build` runs it, and `mise run test` runs it after the Unity tests.

### Parse the binary table directly

The built table is parsed from its documented binary format (32-byte entries: magic `0x50AA`, type, subtype, offset, size, label, flags; ended by an MD5 entry or erased flash), rather than by importing the framework's `gen_esp32part.py`. The format is fixed by the bootloader, the parser is a few lines, and the test then needs no framework install. The project's `partitions.csv` is parsed with a small reader that accepts hex, decimal and `K`/`M` sizes and requires an explicit factory offset and size, which our table has.

### What is compared

1. The built table has a factory app partition (type 0, subtype 0x00).
2. Its offset and size equal those of the `factory` row in `firmware/bridge/partitions.csv`. This catches a table taken from somewhere else, such as the framework's 1 MB default when `board_build.partitions` is missing, even when the image would happen to fit.
3. `$ESP32_APP_OFFSET`, where the upload writes the image, equals the factory offset.
4. The size of `firmware.bin` is at most the factory partition size. Equal passes. The image file is what is written into the partition, so its length is what the bootloader checks.

Sizes are reported in MiB with two decimals (`MB` in the message, as in ESP-IDF's own output), and the free share is rounded down, so a nearly full partition never reads as having more room than it has.

## Risks / Trade-offs

- [A platform update renames `$ESP32_APP_OFFSET` or the build file names] → the script fails loudly when a value or file is missing, rather than skipping the check.
- [SCons resolves the alias before `firmware.bin` exists on a clean build] → the alias lists the image and table as sources, so SCons builds them first.
