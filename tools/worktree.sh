#!/usr/bin/env bash
# Delivery worktrees: a git worktree under .claude/worktrees/<name> on branch
# deliver/<name>, ready for every gate without touching the network.
#
#   tools/worktree.sh add <name> [base]   (base: origin/main after a fetch)
#   tools/worktree.sh remove <name>       (the worktree and its local branch)
#
# The SDK submodules are cloned from this checkout's own (it must have run
# `mise run setup`), so they are the pinned commits and take seconds. The
# bridge's PlatformIO core (.pio-core, git-ignored) is shared by a symlink.
set -euo pipefail

root=$(git rev-parse --show-toplevel)
cmd=${1:-}
name=${2:-}
if [[ -z $cmd || -z $name || ! $name =~ ^[a-z0-9][a-z0-9-]*$ ]]; then
  echo "usage: $0 add <name> [base] | remove <name>   (name: lower-case, digits, dashes)" >&2
  exit 2
fi
dir=$root/.claude/worktrees/$name
branch=deliver/$name

# Clones the listed submodules of the repository at $1 from the same paths
# under the source checkout $2, at the commits the repository pins.
clone_local() {
  local repo=$1 src=$2
  shift 2
  git -C "$repo" submodule init -q "$@"
  for s in "$@"; do
    git -C "$repo" config "submodule.$s.url" "$src/$s"
  done
  git -C "$repo" -c protocol.file.allow=always submodule update -q "$@"
}

case $cmd in
add)
  base=${3:-}
  if [[ -z $base ]]; then
    git -C "$root" fetch -q origin main
    base=origin/main
  fi
  if [[ -e $dir ]]; then
    echo "$dir exists" >&2
    exit 1
  fi
  git -C "$root" worktree add -q -b "$branch" "$dir" "$base"
  clone_local "$dir" "$root" external/pico-sdk external/pimoroni-pico external/bluepad32-template external/LovyanGFX
  clone_local "$dir/external/pico-sdk" "$root/external/pico-sdk" lib/tinyusb
  mapfile -t bp < <(git -C "$dir/external/bluepad32-template" config --file .gitmodules --get-regexp '\.path$' | awk '{print $2}')
  clone_local "$dir/external/bluepad32-template" "$root/external/bluepad32-template" "${bp[@]}"
  if [[ -d $root/.pio-core ]]; then
    ln -s "$root/.pio-core" "$dir/.pio-core"
  fi
  (cd "$dir" && mise trust -q)
  echo "$dir on $branch from $base"
  ;;
remove)
  if [[ -d $dir ]]; then
    git -C "$root" worktree remove --force "$dir"
  fi
  if git -C "$root" show-ref -q --verify "refs/heads/$branch"; then
    git -C "$root" branch -D -q "$branch"
  fi
  echo "removed $name"
  ;;
*)
  echo "unknown command: $cmd" >&2
  exit 2
  ;;
esac
