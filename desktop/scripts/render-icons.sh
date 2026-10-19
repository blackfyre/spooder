#!/bin/sh
# Renders assets/icon.svg to the PNG sizes the Linux icon theme uses and to a
# Windows .ico. Needs resvg and ImageMagick; the outputs are committed, so
# only a change to the SVG needs this.
set -eu
cd "$(dirname "$0")/../assets"
for s in 16 24 32 48 64 128 256 512; do
  resvg -w "$s" -h "$s" icon.svg "icon-$s.png"
done
magick icon-16.png icon-24.png icon-32.png icon-48.png icon-64.png icon-128.png icon-256.png icon.ico
