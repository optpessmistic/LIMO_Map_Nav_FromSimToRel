#!/usr/bin/env bash
# Install the official Noble/Jazzy/Harmonic stack in the selected WSL distro.
# Run as root (or sudo bash). Does not alter shell startup files or upgrade OS.
set -euo pipefail
if [[ $(id -u) != 0 ]]; then
  echo 'Run with sudo bash tools/setup_jazzy.sh' >&2
  exit 1
fi
source /etc/os-release
if [[ ${ID:-} != ubuntu || ${VERSION_CODENAME:-} != noble ]]; then
  echo 'This installer only supports Ubuntu 24.04 Noble.' >&2
  exit 1
fi
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get -o DPkg::Lock::Timeout=300 install -y --no-remove ca-certificates curl locales software-properties-common
add-apt-repository -y universe
locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

# Pinned official ros-infrastructure release; reviewed 2026-10-28.
source_deb=/tmp/limo-ros2-apt-source_1.3.0.noble_all.deb
curl --fail --location --retry 3 --output "$source_deb" \
  https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.noble_all.deb
printf '%s  %s\n' f31d84adf5054c7d60ded0e82c0f776a77ea33af53d08f40b9ad7c94cca55296 "$source_deb" | sha256sum --check -
dpkg -i "$source_deb"
apt-get update
apt-get -o DPkg::Lock::Timeout=300 install -y --no-remove \
  ros-jazzy-desktop ros-dev-tools ros-jazzy-ros-gz \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization ros-jazzy-xacro ros-jazzy-joint-state-publisher \
  ros-jazzy-tf-transformations python3-opencv python3-pil python3-yaml
echo 'Installed. In a fresh terminal: source /opt/ros/jazzy/setup.bash'
