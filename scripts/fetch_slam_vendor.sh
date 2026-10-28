#!/usr/bin/env bash
# Source-only, pinned upstream checkout. Never replaces an existing checkout.
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
vendor_path="$project_root/vendor/slam_toolbox"
upstream_url="https://github.com/SteveMacenski/slam_toolbox.git"
upstream_tag="2.8.5"
upstream_commit="ec8f7635dea317b531c419f798f87d90a336f32e"
header_path="lib/karto_sdk/include/karto_sdk/Karto.h"
patched_header_sha256="3a91277a6fb16bd5796f41d484c2d405384f9bcfef6956f90511fa7c4391512c"
patch_path="$project_root/patches/0002-slam-toolbox-inclusive-grid.patch"
check_only=false
if [[ "${1:-}" == "--check-only" && $# -eq 1 ]]; then
  check_only=true
elif [[ $# -ne 0 ]]; then
  printf 'Usage: %s [--check-only]\n' "$0" >&2
  exit 2
fi

command -v git >/dev/null
command -v sha256sum >/dev/null
git -C "$project_root" check-ignore -q vendor/slam_toolbox || {
  printf 'vendor/slam_toolbox must be ignored by project Git.\n' >&2
  exit 1
}
test -f "$patch_path"
if [[ ! -d "$vendor_path/.git" ]]; then
  if "$check_only" || [[ -e "$vendor_path" ]]; then
    printf 'Missing checkout or pre-existing non-Git directory preserved: %s\n' "$vendor_path" >&2
    exit 1
  fi
  mkdir -p "$project_root/vendor"
  git -c core.autocrlf=false clone --depth 1 --branch "$upstream_tag" \
    --no-checkout "$upstream_url" "$vendor_path"
  [[ "$(git -C "$vendor_path" rev-parse HEAD)" == "$upstream_commit" ]] || {
    printf 'Upstream tag no longer resolves to the pinned commit; refusing checkout.\n' >&2
    exit 1
  }
  git -C "$vendor_path" -c core.autocrlf=false checkout --detach "$upstream_commit"
fi

[[ "$(git -C "$vendor_path" remote get-url origin)" == "$upstream_url" ]] || {
  printf 'Unexpected origin; existing checkout preserved.\n' >&2; exit 1;
}
[[ "$(git -C "$vendor_path" rev-parse HEAD)" == "$upstream_commit" ]] || {
  printf 'Unexpected HEAD; existing checkout preserved.\n' >&2; exit 1;
}
while IFS= read -r untracked_path; do
  [[ "$untracked_path" == "COLCON_IGNORE" ]] || {
    printf 'Unexpected untracked vendor file preserved: %s\n' "$untracked_path" >&2
    exit 1
  }
done < <(git -C "$vendor_path" ls-files --others --exclude-standard)
git -C "$vendor_path" diff --cached --quiet || {
  printf 'Staged vendor changes found; refusing to modify them.\n' >&2; exit 1;
}

if git -C "$vendor_path" diff --quiet; then
  if "$check_only"; then
    printf 'The inclusive grid patch has not been applied.\n' >&2
    exit 1
  fi
  git -C "$vendor_path" apply --check "$patch_path"
  git -C "$vendor_path" apply "$patch_path"
fi
[[ "$(git -C "$vendor_path" diff --name-only)" == "$header_path" ]] || {
  printf 'Unexpected vendor modifications; checkout preserved.\n' >&2; exit 1;
}
git -C "$vendor_path" apply --reverse --check "$patch_path"
[[ "$(sha256sum "$vendor_path/$header_path" | cut -d ' ' -f 1)" == "$patched_header_sha256" ]] || {
  printf 'Patched header differs from the expected two-line change.\n' >&2; exit 1;
}

# Build an explicit source snapshot in outputs/slam-boundary-ws instead. This
# prevents plain colcon discovery from silently changing the main ROS overlay.
if "$check_only"; then
  test -f "$vendor_path/COLCON_IGNORE"
else
  touch "$vendor_path/COLCON_IGNORE"
fi
printf 'Validated upstream: %s @ %s (%s)\n' "$upstream_url" "$upstream_commit" "$upstream_tag"
printf 'Patch: %s\n' "$patch_path"
printf 'Isolated build instructions: docs/slam-grid-boundary.md\n'
