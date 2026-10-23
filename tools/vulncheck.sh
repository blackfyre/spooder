#!/usr/bin/env bash
# govulncheck for desktop/, against a shrink-only baseline.
#
#   tools/vulncheck.sh
#
# Fails on a vulnerability the desktop code calls that desktop/vulncheck-baseline.txt
# does not list, and on a baseline entry no longer found (remove it: the
# baseline only shrinks). Lines in the baseline: "<GO-id> <note>"; # comments.
set -euo pipefail

root=$(git rev-parse --show-toplevel)
baseline=$root/desktop/vulncheck-baseline.txt
version=v1.9.0 # golang.org/x/vuln, pinned

report=$(cd "$root/desktop" && CGO_ENABLED=1 go run "golang.org/x/vuln/cmd/govulncheck@$version" -format json ./...)
# A finding whose trace starts at a function is one the code calls.
found=$(jq -r 'select(.finding) | .finding | select(.trace[0].function != null) | .osv' <<<"$report" | sort -u)
known=$(grep -v '^\s*\(#\|$\)' "$baseline" | awk '{print $1}' | sort -u)

new=$(comm -23 <(printf '%s\n' "$found" | sed '/^$/d') <(printf '%s\n' "$known" | sed '/^$/d'))
gone=$(comm -13 <(printf '%s\n' "$found" | sed '/^$/d') <(printf '%s\n' "$known" | sed '/^$/d'))

status=0
if [[ -n $new ]]; then
  echo "New vulnerabilities the desktop code calls (fix, or add to the baseline in a reviewed commit):"
  mapfile -t ids <<<"$new"
  printf '  %s\n' "${ids[@]}"
  status=1
fi
if [[ -n $gone ]]; then
  echo "Fixed, so remove from desktop/vulncheck-baseline.txt:"
  mapfile -t ids <<<"$gone"
  printf '  %s\n' "${ids[@]}"
  status=1
fi
if ((status == 0)); then
  echo "govulncheck: $(printf '%s\n' "$found" | sed '/^$/d' | wc -l) called vulnerabilities, all in the baseline."
fi
exit $status
