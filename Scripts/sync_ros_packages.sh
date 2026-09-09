#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
target="${project_root}/ros_ws"
readonly url="https://github.com/FPGAGowin2026/ros_ws.git"

# 保留原脚本名作为入口；现在统一同步工作区，不再逐包克隆。

if [[ -e "${target}/.git" ]]; then
  echo "更新 ros_ws"
  git -C "${target}" pull --ff-only
elif [[ -e "${target}" ]]; then
  echo "错误：${target} 已存在，但不是独立 Git 仓库。" >&2
  exit 1
else
  echo "克隆 ros_ws"
  git clone "${url}" "${target}"
fi
