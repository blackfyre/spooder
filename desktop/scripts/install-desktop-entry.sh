#!/bin/sh
# Installs the calibrator's launcher and icon for the current user (Linux):
# the icon into the hicolor theme under ~/.local/share/icons and a desktop
# entry that starts the built calibrator from this checkout, so that its
# export finds the checkout. The entry's name matches Gio's default app ID
# (the binary's name), which Wayland and X11 use to find the icon.
set -eu
[ "$(uname -s)" = Linux ] || { echo "desktop entries are for Linux only"; exit 0; }
root=$(cd "$(dirname "$0")/../.." && pwd)
bin="$root/build/desktop/spooder-calibrator"
[ -x "$bin" ] || { echo "build it first: mise run desktop-build"; exit 1; }
data=${XDG_DATA_HOME:-$HOME/.local/share}
for s in 16 24 32 48 64 128 256 512; do
  install -Dm644 "$root/desktop/assets/icon-$s.png" "$data/icons/hicolor/${s}x$s/apps/spooder-calibrator.png"
done
install -Dm644 "$root/desktop/assets/icon.svg" "$data/icons/hicolor/scalable/apps/spooder-calibrator.svg"
mkdir -p "$data/applications"
cat > "$data/applications/spooder-calibrator.desktop" <<ENTRY
[Desktop Entry]
Type=Application
Name=Spooder calibrator
Comment=Monitor and calibrate the Spooder hexapod over USB
Exec="$bin"
Path=$root
Icon=spooder-calibrator
Terminal=false
Categories=Development;Electronics;
StartupWMClass=spooder-calibrator
ENTRY
command -v update-desktop-database >/dev/null && update-desktop-database "$data/applications" || true
command -v gtk-update-icon-cache >/dev/null && gtk-update-icon-cache -q "$data/icons/hicolor" 2>/dev/null || true
echo "installed $data/applications/spooder-calibrator.desktop"
