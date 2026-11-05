#!/usr/bin/env bash
# Delivery worktrees: a git worktree under .claude/worktrees/<name> on branch
# deliver/<name>, ready for every gate without touching the network.
#
#   tools/worktree.sh add <name> [base]   (base: origin/main after a fetch)
#   tools/worktree.sh remove <name>       (the worktree and its local branch)
#   tools/worktree.sh init                (prepare the worktree it is run in,
#                                          one made by another tool, e.g. Orca)
#
# The SDK submodules are cloned from the primary checkout's own (it must have
# run `mise run setup`), so they are the pinned commits and take seconds. The
# bridge's PlatformIO core (.pio-core, git-ignored) is shared by a symlink.
set -euo pipefail

usage() {
  echo "usage: $0 add <name> [base] | remove <name> | init   (name: lower-case, digits, dashes)" >&2
  exit 2
}

# The primary checkout: the one holding the repository's common git directory.
root=$(dirname "$(git rev-parse --path-format=absolute --git-common-dir)")
cmd=${1:-}
name=${2:-}
if [[ $cmd != init && (-z $name || ! $name =~ ^[a-z0-9][a-z0-9-]*$) ]]; then
  usage
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

# Readies the worktree at $1 for every gate: its SDK submodules from the
# primary checkout's, the shared PlatformIO core, and mise trust.
prepare() {
  local wt=$1
  if [[ ! -e $root/external/pico-sdk/lib/tinyusb/.git ]]; then
    echo "$root has no SDK submodules: run mise run setup there first" >&2
    exit 1
  fi
  clone_local "$wt" "$root" external/pico-sdk external/pimoroni-pico external/bluepad32-template external/LovyanGFX
  clone_local "$wt/external/pico-sdk" "$root/external/pico-sdk" lib/tinyusb
  mapfile -t bp < <(git -C "$wt/external/bluepad32-template" config --file .gitmodules --get-regexp '\.path$' | awk '{print $2}')
  clone_local "$wt/external/bluepad32-template" "$root/external/bluepad32-template" "${bp[@]}"
  if [[ -d $root/.pio-core && ! -e $wt/.pio-core ]]; then
    ln -s "$root/.pio-core" "$wt/.pio-core"
  fi
  (cd "$wt" && mise trust -q)
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
  prepare "$dir"
  echo "$dir on $branch from $base"
  ;;
init)
  here=$(git rev-parse --show-toplevel)
  if [[ $here == "$root" ]]; then
    echo "$here is the primary checkout: use mise run setup" >&2
    exit 1
  fi
  prepare "$here"
  echo "$here ready"
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
  usage
  ;;
esac
