#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
target="${project_root}/FPGA_ws"
readonly url="https://github.com/FPGAGowin2026/fpga_rtl.git"

if [[ -e "${target}/.git" ]]; then
  echo "更新 FPGA_ws"
  git -C "${target}" pull --ff-only
elif [[ -e "${target}" ]]; then
  echo "错误：${target} 已存在，但不是独立 Git 仓库。" >&2
  exit 1
else
  echo "克隆 FPGA_ws"
  git clone "${url}" "${target}"
fi
