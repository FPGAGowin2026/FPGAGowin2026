#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ $# != 2 || -z "${2//[[:space:]]/}" ]]; then
  echo "用法：$0 {main|ugv_ws|uav_ws|swarm_ws|FPGA_ws|llm_ws} \"提交说明\"" >&2
  exit 1
fi

case "$1" in
  gowingame) echo "错误：gowingame 已并入 FPGA_ws，请使用 FPGA_ws 目标。" >&2; exit 1 ;;
  ros_ws) echo "错误：ros_ws 已更名为 ugv_ws，请使用 ugv_ws 目标。" >&2; exit 1 ;;
esac
relative="$(python3 "${project_root}/Scripts/repository.py" "$1")"
target="${project_root}/${relative}"

# 防止 Git 向上查找时误用父仓库；兼容 worktree 的 .git 文件。
if [[ ! -e "${target}/.git" ]]; then
  echo "错误：${target} 不是独立 Git 仓库，请先运行对应同步脚本。" >&2
  exit 1
fi

branch="$(git -C "${target}" symbolic-ref --quiet --short HEAD)" || {
  echo "错误：当前处于 detached HEAD，请先切换到要推送的分支。" >&2
  exit 1
}

# 提交前检查远端配置；连接或推送失败时保留本地提交，方便重试。
git -C "${target}" remote get-url origin >/dev/null
if [[ "$1" == main ]]; then
  paths=(.)
  workspace_list="$(python3 "${project_root}/Scripts/repository.py" --list)"
  mapfile -t workspaces <<< "${workspace_list}"
  for path in "${workspaces[@]}" ros_ws gowingame .repo; do
    if ! git -C "${target}" diff --cached --quiet -- "${path}"; then
      echo "错误：主仓库暂存区包含独立仓库或元数据 ${path}，请自行处理；暂存内容保留。" >&2
      exit 1
    fi
    if ! git -C "${target}" check-ignore -q -- "${path}"; then
      paths+=(":(exclude)${path}")
    fi
  done
  git -C "${target}" add -A -- "${paths[@]}"
else
  git -C "${target}" add -A
fi
git -C "${target}" diff --cached --check
if git -C "${target}" diff --cached --quiet; then
  echo "没有新改动，直接推送已有提交"
else
  git -C "${target}" commit -m "$2"
fi

echo "推送 ${target} 的 ${branch} 分支到 origin"
git -C "${target}" -c push.followTags=false push --set-upstream origin "HEAD:refs/heads/${branch}"
