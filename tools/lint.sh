#!/usr/bin/env bash
# The C++ static checks: the formatting check of Spooder's own C++ and
# clang-tidy on core/. It modifies no source file.
#
#   tools/lint.sh        (mise run lint)
#
# Fails if either check fails, or if the clang tools on PATH are not the
# versions pinned in mise.toml. Reformat with clang-format -i on the files
# the formatting check names.
set -euo pipefail

root=$(git rev-parse --show-toplevel)
cd "$root"

# Spooder's own C++. Exclusions inside these (generated fonts, golden files,
# the exported calibration) are in .clang-format-ignore, which clang-format
# honours for files named on its command line too.
checked_dirs=(core firmware/servo2040/src firmware/bridge/main test tools/virtual-robot tools/bridge-screenshots)

# The version pinned in mise.toml for a pipx: tool.
pinned() {
  sed -n "s/^\"pipx:$1\" *= *\"\([^\"]*\)\".*/\1/p" mise.toml
}

check_version() {
  local tool=$1 want have
  want=$(pinned "$tool")
  if [[ -z $want ]]; then
    echo "lint: no \"pipx:$tool\" pin in mise.toml" >&2
    return 1
  fi
  if ! command -v "$tool" >/dev/null; then
    echo "lint: $tool not found; run mise install, then mise run lint" >&2
    return 1
  fi
  have=$("$tool" --version | sed -n 's/.*version \([0-9][0-9.]*\).*/\1/p' | head -n1)
  if [[ $have != "$want" ]]; then
    echo "lint: $tool is ${have:-unknown}, mise.toml pins $want; run it through mise run lint" >&2
    return 1
  fi
}

# Tracked and untracked-but-not-ignored C++ files in the checked directories,
# so a new file counts before it is staged and build output never does. A
# tracked file deleted from the working tree is skipped.
files=()
while IFS= read -r -d '' f; do
  [[ -f $f ]] && files+=("$f")
done < <(git ls-files -z --cached --others --exclude-standard -- \
  "${checked_dirs[@]/%//*.c}" "${checked_dirs[@]/%//*.cpp}" \
  "${checked_dirs[@]/%//*.h}" "${checked_dirs[@]/%//*.hpp}" | sort -zu)

check_format() {
  check_version clang-format || return 1
  if [[ ! -f .clang-format ]]; then
    echo "lint: no .clang-format at the repository root" >&2
    return 1
  fi
  if ((${#files[@]} == 0)); then
    echo "lint: no C++ files found in ${checked_dirs[*]}" >&2
    return 1
  fi
  # --dry-run reports each unformatted file and changes nothing.
  clang-format --dry-run --Werror "${files[@]}"
}

check_tidy() {
  check_version clang-tidy || return 1
  if [[ ! -f .clang-tidy ]]; then
    echo "lint: no .clang-tidy at the repository root" >&2
    return 1
  fi
  # The host compile database (the native env of the host tests), written in
  # this run so a stale one is never analysed.
  # (errexit does not apply inside a function called with ||, so every step
  # checks its own status.)
  rm -f compile_commands.json || return 1
  if ! pio run -e native -t compiledb >/dev/null; then
    echo "lint: pio run -e native -t compiledb failed" >&2
    return 1
  fi
  if [[ ! -f compile_commands.json ]]; then
    echo "lint: pio run -e native -t compiledb wrote no compile_commands.json" >&2
    return 1
  fi
  local sources=()
  mapfile -t sources < <(python3 - <<'EOF'
import json, os
root = os.getcwd()
for entry in json.load(open("compile_commands.json")):
    path = os.path.relpath(os.path.join(entry["directory"], entry["file"]), root)
    if path.startswith("core" + os.sep):
        print(path)
EOF
  )
  if ((${#sources[@]} == 0)); then
    echo "lint: the compile database lists no core/ source" >&2
    return 1
  fi
  clang-tidy -p . --quiet "${sources[@]}"
}

status=0
echo "== clang-format: ${#files[@]} files"
check_format || status=1
echo "== clang-tidy: core/"
check_tidy || status=1

if ((status == 0)); then
  echo "lint: formatting and clang-tidy clean."
else
  echo "lint: failed." >&2
fi
exit $status
