#!/usr/bin/env bash
set -euo pipefail
repo=/mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
cd -- "$repo"
vendor="$repo/vendor/slam_toolbox"
patch_file="$repo/patches/0003-slam-toolbox-steady-transform-publish.patch"
bash -n scripts/fetch_slam_vendor.sh
git -C "$vendor" apply --check "$patch_file"
before="$(sha256sum "$vendor/lib/karto_sdk/include/karto_sdk/Karto.h" "$vendor/src/slam_toolbox_common.cpp")"
if bash scripts/fetch_slam_vendor.sh --check-only; then
  printf 'ERROR: boundary-only checkout incorrectly passed check-only\n' >&2
  exit 1
fi
[[ "$before" == "$(sha256sum "$vendor/lib/karto_sdk/include/karto_sdk/Karto.h" "$vendor/src/slam_toolbox_common.cpp")" ]]
printf 'PASS: boundary-only check-only refuses without changing either source\n'
probe="$vendor/.limo-fetch-guard-probe"
[[ ! -e "$probe" && ! -L "$probe" ]]
trap 'rm -f -- "$probe"' EXIT
printf 'Owned temporary untracked guard probe\n' > "$probe"
if bash scripts/fetch_slam_vendor.sh; then
  printf 'ERROR: unexpected untracked file was accepted\n' >&2
  exit 1
fi
test -f "$probe"
[[ "$before" == "$(sha256sum "$vendor/lib/karto_sdk/include/karto_sdk/Karto.h" "$vendor/src/slam_toolbox_common.cpp")" ]]
rm -- "$probe"
trap - EXIT
printf 'PASS: unexpected file preserved and no missing patch applied\n'
bash scripts/fetch_slam_vendor.sh
after="$(sha256sum "$vendor/lib/karto_sdk/include/karto_sdk/Karto.h" "$vendor/src/slam_toolbox_common.cpp")"
bash scripts/fetch_slam_vendor.sh
bash scripts/fetch_slam_vendor.sh --check-only
[[ "$after" == "$(sha256sum "$vendor/lib/karto_sdk/include/karto_sdk/Karto.h" "$vendor/src/slam_toolbox_common.cpp")" ]]
git -C "$vendor" apply --reverse --check "$patch_file"
git -C "$vendor" diff --check
printf 'PASS: apply, repeated apply, check-only and reverse applicability\n%s\n' "$after"
git -C "$vendor" diff --numstat
