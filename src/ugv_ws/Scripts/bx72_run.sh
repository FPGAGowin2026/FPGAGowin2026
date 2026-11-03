#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$workspace"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
exec ros2 launch car_bringup car_bringup.launch.py "$@"
