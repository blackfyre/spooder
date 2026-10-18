#!/bin/sh
# Checks the C toolchain and the pkg-config entries that Gio's cgo directives
# need on Linux (gioui.org v0.10.3: app/os_wayland.go, app/os_x11.go,
# app/egl_wayland.go, app/internal/xkb, internal/egl, internal/vk), and names
# the Fedora packages that provide whatever is missing.
set -u

[ "$(uname -s)" = Linux ] || exit 0

missing=""
add() {
  case " $missing " in
    *" $1 "*) ;;
    *) missing="$missing $1" ;;
  esac
}

command -v cc >/dev/null 2>&1 || command -v gcc >/dev/null 2>&1 || add gcc
if ! command -v pkg-config >/dev/null 2>&1; then
  add pkgconf-pkg-config
  pc=false
else
  pc=true
fi

# pkg-config name -> Fedora package
for entry in \
  wayland-client:wayland-devel \
  wayland-cursor:wayland-devel \
  wayland-egl:wayland-devel \
  egl:mesa-libEGL-devel \
  xkbcommon:libxkbcommon-devel \
  xkbcommon-x11:libxkbcommon-x11-devel \
  x11:libX11-devel \
  x11-xcb:libX11-devel \
  xcursor:libXcursor-devel \
  xfixes:libXfixes-devel; do
  name=${entry%%:*}
  pkg=${entry#*:}
  if [ "$pc" = false ] || ! pkg-config --exists "$name"; then
    add "$pkg"
  fi
done

# Gio's Vulkan backend includes <vulkan/vulkan.h> without pkg-config.
if ! printf '#include <vulkan/vulkan.h>\n' | ${CC:-cc} -E -x c - >/dev/null 2>&1; then
  add vulkan-headers
fi

if [ -n "$missing" ]; then
  echo "The desktop calibrator's Gio build needs these packages:" >&2
  echo "  sudo dnf install$missing" >&2
  exit 1
fi
