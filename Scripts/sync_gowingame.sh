#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
target="${project_root}/gowingame"
readonly url="https://github.com/FPGAGowin2026/gowingame.git"

if [[ -e "${target}/.git" ]]; then
  echo "更新 gowingame"
  git -C "${target}" pull --ff-only
elif [[ -e "${target}" ]]; then
  echo "错误：${target} 已存在，但不是独立 Git 仓库。" >&2
  exit 1
else
  echo "克隆 gowingame"
  git clone "${url}" "${target}"
fi
