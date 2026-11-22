#!/usr/bin/env bash
# Raspberry Pi entry point for the ACG720 FPGA bridge.
set -eo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-}" in
  -h|--help)
    printf 'Usage: bash Scripts/acg720_run.sh [bridge] [bridge or ROS arguments]\n'
    printf '       bash Scripts/acg720_run.sh build\n'
    exit 0 ;;
  build)
    source /opt/ros/humble/setup.bash
    cd "$workspace"
    exec colcon build --packages-select legged_bridge --executor sequential ;;
  bridge) shift ;;
esac
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_FASTRTPS_PUBLICATION_MODE="${RMW_FASTRTPS_PUBLICATION_MODE:-ASYNCHRONOUS}"
export PYTHONDONTWRITEBYTECODE=1
if systemctl --user is-active --quiet acg720-bridge.service 2>/dev/null ||
   pgrep -u "$UID" -f '(^|[[:space:]/])legged_bridge[/.]acg720_bridge(\.py)?([[:space:]]|$)' >/dev/null; then
  printf 'ACG720 bridge is already running.\n'; exit 0
fi
if (($# == 0)) && [[ $(systemctl --user show -p LoadState --value acg720-bridge.service 2>/dev/null) == loaded ]]; then
  exec systemctl --user start acg720-bridge.service
fi
for arg in "$@"; do
  if [[ $arg == --http-only ]]; then
    export PYTHONPATH="$workspace/src/legged/legged_bridge${PYTHONPATH:+:$PYTHONPATH}"
    exec python3 -m legged_bridge.acg720_bridge "$@"
  fi
done
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
exec ros2 run legged_bridge acg720_bridge --ros-args -p lidar_self_radius_m:=0.20 -- "$@"
