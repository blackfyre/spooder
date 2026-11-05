#!/bin/sh
# Spooder.pak's launch script: NextUI runs it from Tools/tg5040/Spooder.pak/
# (openspec change add-handheld-controller, design D6). It reads wlan0's WiFi
# power-saving setting, runs spooder-handheld --device (which turns power
# saving off for the run and restores it itself), and restores the setting
# however the binary ends: a normal exit, an error, a crash, or a signal to
# this script, which also stops the binary and any process it left running.
# Restoring twice is harmless.

PAK_DIR=$(cd "$(dirname "$0")" && pwd)
cd "$PAK_DIR" || exit 1
PATH="$PAK_DIR:$PATH"
# The device's own libraries win; the pak's (libSDL2_gfx) fill the gap.
export LD_LIBRARY_PATH="/usr/trimui/lib:/usr/lib:$PAK_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [ -n "${LOGS_PATH:-}" ] && [ -d "$LOGS_PATH" ]; then
  exec >>"$LOGS_PATH/Spooder.txt" 2>&1
fi

# "Power save: on" or "Power save: off"; anything else leaves the setting
# alone.
saved=$(iw dev wlan0 get power_save 2>/dev/null | sed -n -E 's/^.*: *(on|off) *$/\1/p')
group=""

# stop ends every process the binary started, and the binary if it still
# runs: the binary leads a process group of its own (setsid below), so one
# signal to the group reaches its children and theirs, whether the binary
# has exited, crashed or is still running.
stop() {
  [ -n "$group" ] || return 0
  kill -TERM -- "-$group" 2>/dev/null || kill -TERM "$group" 2>/dev/null
  # A binary that ignores SIGTERM or hangs on its way out gets SIGKILL after
  # STOP_GRACE seconds, so the setting is restored all the same.
  # Cancelled once the binary has gone: its sleep is stopped with it, and it
  # holds none of the script's output, so nothing outlives the script.
  (
    exec </dev/null >/dev/null 2>&1
    sleep "${STOP_GRACE:-2}" &
    timer=$!
    trap 'kill "$timer"; exit 0' TERM
    wait "$timer" || exit 0
    kill -KILL -- "-$group" || kill -KILL "$group"
  ) &
  watchdog=$!
  wait "$group" 2>/dev/null
  kill "$watchdog" 2>/dev/null
  wait "$watchdog" 2>/dev/null
  # Anything of the group that ignored SIGTERM.
  kill -KILL -- "-$group" 2>/dev/null
  group=""
}

restore() {
  stop
  if [ -n "$saved" ]; then
    iw dev wlan0 set power_save "$saved"
  fi
}
# restore runs once: on the way out, or for a signal, which then exits.
trap restore EXIT
trap 'restore; trap - EXIT; exit 129' HUP
trap 'restore; trap - EXIT; exit 130' INT
trap 'restore; trap - EXIT; exit 143' TERM

# In the background, so that a signal to this script runs its trap at once
# instead of after the binary has exited. setsid makes the binary (same pid:
# a background job of this script is no group leader, so setsid does not
# fork) the leader of a new process group.
if command -v setsid >/dev/null 2>&1; then
  setsid spooder-handheld --device "$@" &
else
  spooder-handheld --device "$@" &
fi
group=$!
wait "$group"
status=$?
exit "$status"
