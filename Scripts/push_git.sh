#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ $# != 2 || -z "${2//[[:space:]]/}" ]]; then
  echo "用法：$0 {main|ros_ws|FPGA_ws|gowingame} \"提交说明\"" >&2
  exit 1
fi

case "$1" in
  main) target="${project_root}" ;;
  ros_ws|FPGA_ws|gowingame) target="${project_root}/$1" ;;
  *) echo "错误：未知仓库 $1" >&2; exit 1 ;;
esac

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
git -C "${target}" add -A
if git -C "${target}" diff --cached --quiet; then
  echo "没有新改动，直接推送已有提交"
else
  git -C "${target}" commit -m "$2"
fi

echo "推送 ${target} 的 ${branch} 分支到 origin"
git -C "${target}" push --set-upstream origin "HEAD:refs/heads/${branch}"
