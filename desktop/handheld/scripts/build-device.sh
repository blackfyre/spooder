#!/bin/sh
# The handheld's device build (design D8): builds the Containerfile's image
# under rootless Podman for linux/arm64 (qemu-user-static), then, inside it,
# the binary to build/handheld/arm64/spooder-handheld with the repository and
# the patched gabagool mounted, copies the image's libSDL2_gfx beside it
# (build/handheld/arm64/lib) and checks the binary with check-glibc.sh.
# Run from the repository root after handheld-fork (mise run
# handheld-build-device).
set -eu

root=$(pwd)
here=$root/desktop/handheld
containerfile=$here/build/Containerfile
out=$root/build/handheld/arm64
cache=$root/build/handheld/arm64-cache

"$here/scripts/check-go-version.sh" "$containerfile" "$root/mise.toml"

# The image is tagged by the Containerfile's hash, so a changed Containerfile
# builds a new one and an unchanged one is reused.
tag=spooder-handheld-build:$(sha256sum "$containerfile" | cut -c1-16)
if ! podman image exists "$tag"; then
  podman build --platform linux/arm64 -t "$tag" -f "$containerfile" "$here/build"
fi

mkdir -p "$out/lib" "$cache"
# keep-id: the outputs belong to the user; :z for SELinux hosts. GOFLAGS:
# the module graph is go.mod's and go.sum's only; -modcacherw so that
# mise run clean can remove the cache; -buildvcs=false since a
# delivery worktree's .git points outside the mount. HOME: the go command
# keeps its telemetry counts there, out of the checkout.
podman run --rm --platform linux/arm64 --userns=keep-id \
  -v "$root:/src:z" -w /src/desktop/handheld \
  -e GOFLAGS="-mod=readonly -buildvcs=false -modcacherw" \
  -e GOCACHE=/src/build/handheld/arm64-cache/go-build \
  -e GOMODCACHE=/src/build/handheld/arm64-cache/mod \
  -e GOPATH=/src/build/handheld/arm64-cache/gopath \
  -e HOME=/src/build/handheld/arm64-cache/home \
  "$tag" sh -c '
    set -e
    go build -trimpath -o /src/build/handheld/arm64/spooder-handheld ./cmd/spooder-handheld
    cp -L /usr/lib/aarch64-linux-gnu/libSDL2_gfx-1.0.so.0 /src/build/handheld/arm64/lib/
  '

"$here/scripts/check-glibc.sh" "$out/spooder-handheld"
echo "Libraries the binary needs:"
readelf -d "$out/spooder-handheld" | sed -n 's/.*(NEEDED).*\[\(.*\)\]/  \1/p'
