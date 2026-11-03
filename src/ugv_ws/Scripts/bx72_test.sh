#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$workspace"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
# 不继承在线节点的ROS域，避免测试发布到实际控制图。
export ROS_DOMAIN_ID="${BX72_TEST_DOMAIN_ID:-95}"
colcon test --base-paths src --packages-select ugv_bridge car_mission car_navigation car_bringup "$@"
exec colcon test-result --verbose
