#!/usr/bin/env bash
# Source-only, pinned upstream checkout. Never replaces an existing checkout.
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
vendor_path="$project_root/vendor/slam_toolbox"
upstream_url="https://github.com/SteveMacenski/slam_toolbox.git"
upstream_tag="2.8.5"
upstream_commit="ec8f7635dea317b531c419f798f87d90a336f32e"
header_path="lib/karto_sdk/include/karto_sdk/Karto.h"
original_header_sha256="277fd3ddbb4c655408660e7bab8ff39f49875af709bd7a83ff3d029190d44052"
patched_header_sha256="3a91277a6fb16bd5796f41d484c2d405384f9bcfef6956f90511fa7c4391512c"
source_path="src/slam_toolbox_common.cpp"
original_source_sha256="7ee09d3669435b1936e1c6aca5e4d1d916d5712c08ba00148d9f8e96a612a9d0"
transform_patched_source_sha256="c1b96ccf0e516cc83dc7628047c0114b489f566d0516c0c3291b7310f7cddfdf"
patched_source_sha256="2c80e9d4af5974f449473bd0b568d00fac0be2aa24e6b97161548766529ac406"
patches=(
  "$project_root/patches/0002-slam-toolbox-inclusive-grid.patch"
  "$project_root/patches/0003-slam-toolbox-steady-transform-publish.patch"
  "$project_root/patches/0004-slam-toolbox-steady-map-publish.patch"
)
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
for patch_path in "${patches[@]}"; do
  test -f "$patch_path"
done
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

# Validate the entire working tree before applying any missing patch. Accept
# only exact prefixes: pristine, 0002, 0002+0003, or 0002+0003+0004. A matching
# hunk alone is insufficient: unrelated edits in either file must be preserved.
while IFS= read -r changed_path; do
  [[ "$changed_path" == "$header_path" || "$changed_path" == "$source_path" ]] || {
    printf 'Unexpected vendor modification preserved: %s\n' "$changed_path" >&2
    exit 1
  }
done < <(git -C "$vendor_path" diff --name-only)
[[ -z "$(git -C "$vendor_path" diff --summary)" ]] || {
  printf 'Unexpected vendor file mode/type changes; checkout preserved.\n' >&2; exit 1;
}
header_sha256="$(sha256sum "$vendor_path/$header_path" | cut -d ' ' -f 1)"
source_sha256="$(sha256sum "$vendor_path/$source_path" | cut -d ' ' -f 1)"
case "$header_sha256:$source_sha256" in
  "$original_header_sha256:$original_source_sha256") applied=0 ;;
  "$patched_header_sha256:$original_source_sha256") applied=1 ;;
  "$patched_header_sha256:$transform_patched_source_sha256") applied=2 ;;
  "$patched_header_sha256:$patched_source_sha256") applied=3 ;;
  *) printf 'Unknown or out-of-order vendor patch state; checkout preserved.\n' >&2; exit 1 ;;
esac
for ((i=0; i<applied; i++)); do
  git -C "$vendor_path" apply --reverse --check "${patches[i]}"
done
if "$check_only" && ((applied != ${#patches[@]})); then
  printf 'All three pinned SLAM patches are required; %s of %s applied.\n' "$applied" "${#patches[@]}" >&2
  exit 1
fi
# Preflight every missing patch before the first write. 0002 changes the header;
# 0003 and 0004 change separate, non-overlapping source hunks. Each check works
# directly on every accepted earlier prefix, including the pristine tree.
for ((i=applied; i<${#patches[@]}; i++)); do
  git -C "$vendor_path" apply --check "${patches[i]}"
done
for ((i=applied; i<${#patches[@]}; i++)); do
  git -C "$vendor_path" apply "${patches[i]}"
done
[[ "$(git -C "$vendor_path" diff --name-only)" == "$header_path"$'\n'"$source_path" &&
   "$(sha256sum "$vendor_path/$header_path" | cut -d ' ' -f 1)" == "$patched_header_sha256" &&
   "$(sha256sum "$vendor_path/$source_path" | cut -d ' ' -f 1)" == "$patched_source_sha256" ]] || {
  printf 'Vendor result differs from the exact three pinned patches; inspect before proceeding.\n' >&2
  exit 1
}

# Build an explicit source snapshot in a separate outputs workspace instead. This
# prevents plain colcon discovery from silently changing the main ROS overlay.
if "$check_only"; then
  test -f "$vendor_path/COLCON_IGNORE"
else
  touch "$vendor_path/COLCON_IGNORE"
fi
printf 'Validated upstream: %s @ %s (%s)\n' "$upstream_url" "$upstream_commit" "$upstream_tag"
printf 'Patch: %s\n' "${patches[@]}"
printf 'Isolated overlay / validation scope: patches/README.md\n'
