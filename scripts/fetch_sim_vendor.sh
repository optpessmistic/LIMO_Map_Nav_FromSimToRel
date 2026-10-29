#!/usr/bin/env bash
# Fetch source only; never run upstream bootstrap, install, or hardware scripts.
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
vendor_path="$project_root/vendor/hunter_gazebo"
upstream_url="https://github.com/nevlife/hunter_gazebo.git"
upstream_commit="8e1d4aeb459e77d549262ca20cac819d9c7a32ee"
patch_path="$project_root/patches/0001-limo-harmonic-project-interfaces.patch"
check_only=false
if [[ "${1:-}" == "--check-only" ]]; then
  check_only=true
elif [[ $# -ne 0 ]]; then
  printf 'Usage: %s [--check-only]\n' "$0" >&2
  exit 2
fi

command -v git >/dev/null
git -C "$project_root" check-ignore -q vendor/hunter_gazebo || {
  printf 'vendor/hunter_gazebo must be ignored by the project Git repository.\n' >&2
  exit 1
}
test -f "$patch_path"
if [[ ! -d "$vendor_path/.git" ]]; then
  if "$check_only"; then
    printf 'Vendor checkout is missing: %s\n' "$vendor_path" >&2
    exit 1
  fi
  if [[ -e "$vendor_path" ]]; then
    printf 'Refusing to replace a pre-existing non-Git directory: %s\n' "$vendor_path" >&2
    exit 1
  fi
  mkdir -p "$project_root/vendor"
  git clone --filter=blob:none --no-checkout "$upstream_url" "$vendor_path"
  git -C "$vendor_path" sparse-checkout set src/limo_base src/gazebo_harmonic/urdf
  git -C "$vendor_path" checkout "$upstream_commit"
fi

actual_origin="$(git -C "$vendor_path" remote get-url origin)"
if [[ "$actual_origin" != "$upstream_url" ]]; then
  printf 'Unexpected vendor origin: %s\n' "$actual_origin" >&2
  exit 1
fi
actual_commit="$(git -C "$vendor_path" rev-parse HEAD)"
if [[ "$actual_commit" != "$upstream_commit" ]]; then
  printf 'Vendor HEAD is %s; expected %s. Existing checkout was preserved.\n' \
    "$actual_commit" "$upstream_commit" >&2
  exit 1
fi

if git -C "$vendor_path" apply --reverse --check "$patch_path" 2>/dev/null; then
  printf 'Vendor commit and project interface patch are already present.\n'
elif "$check_only"; then
  printf 'Project interface patch is missing or differs from the expected patch.\n' >&2
  exit 1
else
  git -C "$vendor_path" apply --check "$patch_path"
  git -C "$vendor_path" apply "$patch_path"
  printf 'Applied project interface patch.\n'
fi

# The sparse sensor directory retains its upstream package.xml. It is an asset
# source for our model-only package, not a package to build with Hunter/RGL deps.
ignore_path="$vendor_path/src/gazebo_harmonic/COLCON_IGNORE"
if "$check_only"; then
  test -f "$ignore_path" || {
    printf 'Missing COLCON_IGNORE for the upstream sensor asset directory.\n' >&2
    exit 1
  }
else
  touch "$ignore_path"
fi

printf 'Model package: limo_harmonic_description\n'
printf 'Upstream: %s @ %s\n' "$upstream_url" "$upstream_commit"
printf 'Build from project root: colcon build --base-paths src vendor/hunter_gazebo/src/limo_base\n'
