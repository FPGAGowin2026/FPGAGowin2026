#!/usr/bin/env bash
set -euo pipefail

uav_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
repository='https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git'
revision='23a8d5a191711dd65633df689bd00f55d4dea8f9'
checkout="${uav_root}/third_party/ego-planner-swarm"
patch_file="${uav_root}/third_party/ego_swarm_humble.patch"

if [[ ! -e "${checkout}" ]]; then
  git clone --filter=blob:none --no-checkout -- "${repository}" "${checkout}"
  git -C "${checkout}" fetch --depth 1 origin "${revision}"
  git -C "${checkout}" checkout --detach "${revision}"
elif [[ ! -d "${checkout}/.git" ]]; then
  printf 'Refusing to use non-Git directory: %s\n' "${checkout}" >&2
  exit 1
fi

if [[ "$(git -C "${checkout}" rev-parse HEAD)" != "${revision}" ]]; then
  printf 'Refusing to alter checkout with unexpected HEAD: %s\n' "${checkout}" >&2
  exit 1
fi
if [[ -n "$(git -C "${checkout}" ls-files --others --exclude-standard)" ]]; then
  printf 'Refusing to alter checkout containing untracked files: %s\n' "${checkout}" >&2
  exit 1
fi
if ! git -C "${checkout}" diff --cached --quiet; then
  printf 'Refusing to alter checkout containing staged changes: %s\n' "${checkout}" >&2
  exit 1
fi

current_diff="$(mktemp)"
trap 'rm -f -- "${current_diff}"' EXIT
git -C "${checkout}" diff --binary --no-ext-diff > "${current_diff}"
if [[ ! -s "${current_diff}" ]]; then
  git -C "${checkout}" apply --check "${patch_file}"
  git -C "${checkout}" apply "${patch_file}"
elif ! cmp -s -- "${current_diff}" "${patch_file}"; then
  printf 'Refusing to replace local changes in: %s\n' "${checkout}" >&2
  exit 1
fi

printf 'EGO-Swarm %s and local patch ready: %s\n' "${revision}" "${checkout}"
