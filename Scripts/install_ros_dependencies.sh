#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly external_root="${project_root}/ros_ws/src/external"
readonly install_root="${external_root}/ros_humble_overlay"
readonly packages=(
  ros-humble-robot-localization
  ros-humble-slam-toolbox
  ros-humble-nav2-bringup
)

if [[ "$(. /etc/os-release && printf '%s' "${VERSION_CODENAME:-}")" != "jammy" ]]; then
  echo "错误：本地依赖覆盖层只支持 Ubuntu 22.04 Jammy。" >&2
  exit 1
fi

download_root="$(mktemp -d)"
trap 'rm -rf -- "${download_root}"' EXIT
mkdir -p "${download_root}/archives/partial" "${install_root}"
touch "${external_root}/COLCON_IGNORE"

echo "下载 ROS 2 Humble 导航依赖（不写入系统目录）"
apt-get --download-only --no-install-recommends -y \
  -o Debug::NoLocking=1 \
  -o Dir::Cache="${download_root}" \
  -o Dir::Cache::archives="${download_root}/archives" \
  install "${packages[@]}"

shopt -s nullglob
debs=("${download_root}/archives/"*.deb)
if ((${#debs[@]} == 0)); then
  echo "错误：没有下载到依赖包，请检查 ROS 2 apt 软件源。" >&2
  exit 1
fi

for deb in "${debs[@]}"; do
  dpkg-deb -x "${deb}" "${install_root}"
done

# 本目录由 Scripts/setup_ros_dependencies.bash 激活；该占位文件避免 colcon 将它误报为损坏前缀。
touch "${install_root}/opt/ros/humble/local_setup.bash"

echo "依赖已安装到 ${install_root}"
echo "继续执行：source Scripts/setup_ros_dependencies.bash"
