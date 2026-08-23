#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
robot_dir="${project_root}/ros_ws/src/robot"
readonly organization="FPGAGowin2026"
readonly packages=(fpga_gateway robot_bringup)

mkdir -p "${robot_dir}"

for package in "${packages[@]}"; do
  target="${robot_dir}/${package}"
  url="https://github.com/${organization}/${package}.git"

  if [[ -e "${target}/.git" ]]; then
    echo "更新 ${package}"
    git -C "${target}" pull --ff-only
  elif [[ -e "${target}" ]]; then
    echo "错误：${target} 已存在，但不是独立 Git 仓库。" >&2
    exit 1
  else
    echo "克隆 ${package}"
    git clone "${url}" "${target}"
  fi
done
