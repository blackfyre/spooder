#!/bin/sh
# Zips Spooder.pak (design D8) from the device build: launch.sh, the arm64
# binary and lib/ (libSDL2_gfx), to build/handheld/Spooder.pak.zip. Run from
# the repository root after handheld-build-device (mise run handheld-package).
set -eu

root=$(pwd)
arm64=$root/build/handheld/arm64
stage=$root/build/handheld/pak
zip=$root/build/handheld/Spooder.pak.zip

for f in "$arm64/spooder-handheld" "$arm64/lib/libSDL2_gfx-1.0.so.0"; do
  [ -f "$f" ] || { echo "package: $f is missing; run mise run handheld-build-device" >&2; exit 1; }
done

rm -rf "$stage" "$zip"
mkdir -p "$stage/Spooder.pak/lib"
cp "$root/desktop/handheld/pak/launch.sh" "$stage/Spooder.pak/launch.sh"
cp "$arm64/spooder-handheld" "$stage/Spooder.pak/spooder-handheld"
cp "$arm64/lib/libSDL2_gfx-1.0.so.0" "$stage/Spooder.pak/lib/"
chmod 755 "$stage/Spooder.pak/launch.sh" "$stage/Spooder.pak/spooder-handheld"
(cd "$stage" && zip -q -r -X "$zip" Spooder.pak)
unzip -l "$zip"
