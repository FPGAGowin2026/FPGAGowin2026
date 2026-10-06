#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# == 0 ]]; then
  repository_list="$(python3 "${project_root}/Scripts/repository.py" --list)"
  mapfile -t workspaces <<< "${repository_list}"
  set -- "${workspaces[@]}"
fi
for workspace in "$@"; do
  [[ "${workspace}" != main ]] || { echo "主仓库请使用 git pull --ff-only。" >&2; exit 1; }
  python3 "${project_root}/Scripts/repository.py" "${workspace}" >/dev/null
done
for workspace in "$@"; do
  relative="$(python3 "${project_root}/Scripts/repository.py" "${workspace}")"
  url="$(python3 "${project_root}/Scripts/repository.py" --url "${workspace}")"
  target="${project_root}/${relative}"
  if [[ -e "${target}/.git" ]]; then
    git -C "${target}" pull --ff-only
  elif [[ -e "${target}" ]]; then
    echo "错误：${target} 已存在，但不是独立 Git 仓库；保留原目录。" >&2
    exit 1
  else
    git clone "${url}" "${target}"
  fi
done
