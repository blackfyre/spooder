#!/usr/bin/env bash
# A change's risk tier (REVIEW.md), from the paths it touches.
#
#   tools/risk-tier.sh <base> [head]   (paths changed from base to head, default HEAD)
#   tools/risk-tier.sh -               (paths on standard input, one per line)
#
# Prints the tier (low, medium or high) on the first line, then each path
# with its tier. The highest path sets the change's tier.
set -euo pipefail

if [[ ${1:-} == - ]]; then
  mapfile -t paths
elif [[ -n ${1:-} ]]; then
  # No rename detection: a moved file counts at its old path and its new one.
  changed=$(git diff --name-only --no-renames "$1" "${2:-HEAD}")
  mapfile -t paths <<<"$changed"
else
  echo "usage: $0 <base> [head] | -" >&2
  exit 2
fi

tier_of() {
  case $1 in
  core/protection/* | core/protocol/* | core/robot/robot_tick.* | core/setup/* | \
    firmware/servo2040/src/servo_output* | firmware/servo2040/src/power_* | \
    firmware/servo2040/src/config.h | firmware/servo2040/src/link* | firmware/servo2040/src/usb_link* | \
    firmware/servo2040/src/main.cpp | firmware/servo2040/CMakeLists.txt | \
    firmware/bridge/main/setup_mode* | firmware/bridge/main/portal_page.h | firmware/bridge/main/link.* | firmware/bridge/main/main.c | \
    firmware/bridge/partitions.csv | firmware/bridge/sdkconfig.defaults | firmware/bridge/platformio.ini | \
    firmware/bridge/dependencies.lock | firmware/bridge/CMakeLists.txt | firmware/bridge/main/CMakeLists.txt | \
    firmware/bridge/scripts/* | platformio.ini | \
    desktop/internal/protocol/* | desktop/internal/link/* | \
    external/* | .gitmodules | mise.toml | desktop/go.mod | desktop/go.sum | \
    .claude/workflows/* | tools/worktree.sh | tools/risk-tier.sh | .github/* | REVIEW.md)
    echo high ;;
  CLAUDE.md | */CLAUDE.md | .claude/skills/*)
    echo medium ;; # instructions that steer later agents
  *.md | openspec/* | hardware/* | docs/* | .archify/*)
    echo low ;;
  *)
    echo medium ;;
  esac
}

rank() { case $1 in low) echo 0 ;; medium) echo 1 ;; high) echo 2 ;; esac }

tier=low
lines=()
for p in "${paths[@]}"; do
  [[ -z $p ]] && continue
  t=$(tier_of "$p")
  lines+=("$t $p")
  if (($(rank "$t") > $(rank "$tier"))); then
    tier=$t
  fi
done
echo "$tier"
if ((${#lines[@]})); then
  printf '%s\n' "${lines[@]}"
fi
