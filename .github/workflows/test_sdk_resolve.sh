#!/usr/bin/env bash
# Local test for the "Resolve openlifu-sdk (signer) release" workflow logic.
# Runs the exact selection against the real repo via gh, for each ref type.
#
#   bash .github/workflows/test_sdk_resolve.sh
#
# Requires an authenticated `gh`. Does NOT install anything.
set -uo pipefail

REPO="OpenwaterHealth/openlifu-sdk"

resolve() {
  # Mirrors the workflow step verbatim.
  local ref="$1" SEL KIND TAG
  if [[ "$ref" == "next" || "$ref" =~ -rc\.[0-9]+$ ]]; then
    SEL='map(select(.isPrerelease)) | .[0].tagName // empty'
    KIND="pre-release"
  else
    SEL='map(select(.isPrerelease | not)) | .[0].tagName // empty'
    KIND="release"
  fi
  TAG=$(gh release list --repo "$REPO" --json tagName,isPrerelease --jq "$SEL")
  printf '%-14s -> %-12s %s\n' "$ref" "$KIND" "${TAG:-<none>}"
}

echo "== gh sees these releases (newest first) =="
gh release list --repo "$REPO" --json tagName,isPrerelease \
  --jq '.[] | "\(.tagName)\t\(if .isPrerelease then "pre-release" else "release" end)"' | head -8
echo
echo "== resolution by ref =="
resolve "next"          # branch build -> pre-release
resolve "main"          # branch build -> release
resolve "1.2.6-rc.5"    # rc tag build -> pre-release  (the failing case)
resolve "1.2.6"         # release tag  -> release
