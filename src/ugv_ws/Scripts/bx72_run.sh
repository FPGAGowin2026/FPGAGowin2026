#!/usr/bin/env bash
# Jetson entry point for car missions, ACG720 mapping and the dashboard.
set -eo pipefail
if [[ ${1:-} == --car-node ]]; then
  shift
  # Keep a single instance even while DDS discovery is incomplete. The lock
  # follows the node across exec and is released when that process exits.
  lock="${XDG_RUNTIME_DIR:-/run/user/$UID}/bx72-car-${ROS_DOMAIN_ID:-0}-${1##*/}.lock"
  exec flock --nonblock --conflict-exit-code 0 --no-fork "$lock" "$@"
fi
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
project="$(cd -- "$workspace/.." && pwd)"
mode=all
case "${1:-}" in
  all|cooperative|car|mission|mapping|build|test) mode=$1; shift ;;
  -h|--help)
    cat <<'USAGE'
bash Scripts/bx72_run.sh [all|car|mission|mapping|build|test] [name:=value ...]
  all, cooperative  Car tasks, ACG720 mapping, cooperative console and dashboard (default)
  car               Car, vision, mapping, waypoint control and dashboard
  mission           Waypoint and command nodes with an existing car mapping session
  mapping           ACG720 mapping on Jetson
  build, test       Explicit development operations
USAGE
    exit 0 ;;
  ''|*:=*) ;;
  *) printf 'Unknown mode: %s\n' "$1" >&2; exit 2 ;;
esac
source /opt/ros/humble/setup.bash
if [[ $mode == build ]]; then
  export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-1}"
  (cd "$workspace" && colcon build --packages-up-to car_bringup legged_bringup --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release)
  source "$workspace/install/setup.bash"
  cd "$project/swarm_ws"
  cmake_args=(-DCMAKE_BUILD_TYPE=Release)
  # The cooperative console loads Humble's cv_bridge in the same process as _april.
  cv_dir="$project/thirdparty/opencv_humble_sdk/cmake"
  if [[ -f $cv_dir/OpenCVConfig.cmake ]]; then cmake_args+=("-DOpenCV_DIR=$cv_dir"); fi
  exec colcon build --packages-select swarm_cooperation --executor sequential --cmake-args "${cmake_args[@]}"
fi
source "$workspace/install/setup.bash"
if [[ $mode == test ]]; then
  cd "$workspace"
  colcon test --packages-select ugv_bridge car_navigation car_bringup car_mission --executor sequential
  exec colcon test-result --verbose
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_FASTRTPS_PUBLICATION_MODE="${RMW_FASTRTPS_PUBLICATION_MODE:-ASYNCHRONOUS}"
ulimit -S -s 125000
if [[ $mode == all || $mode == cooperative || $mode == mapping ]]; then
  runtime_env="${ACG720_RTABMAP_ENV:-$project/thirdparty/rtabmap_runtime/env.sh}"
  if [[ -f $runtime_env ]]; then source "$runtime_env"; fi
fi
if [[ $mode == all || $mode == cooperative ]]; then source "$project/swarm_ws/install/setup.bash"; fi

# Discover once, before starting any launch. Reused nodes belong to their original launcher.
node_list="$(ros2 node list --no-daemon --spin-time 3)"
has_node() { [[ $'\n'"$node_list"$'\n' == *$'\n'"$1"$'\n'* ]]; }
set -m
children=()
start() { "$@" & children+=("$!"); }
cleanup() {
  trap '' INT TERM HUP
  local pid
  for pid in "${children[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

car_args=()
mapping_args=()
swarm_args=()
members=car,acg720
for arg in "$@"; do
  if [[ $mode == mapping ]]; then mapping_args+=("$arg"); continue; fi
  if [[ $mode == car || $mode == mission ]]; then car_args+=("$arg"); continue; fi
  case "$arg" in
    participants:=*) members=${arg#*:=}; swarm_args+=("$arg") ;;
    acg_camera_mount_file:=*) mapping_args+=("camera_mount_file:=${arg#*:=}"); swarm_args+=("$arg") ;;
    acg_camera_info_file:=*) mapping_args+=("camera_info_file:=${arg#*:=}"); swarm_args+=("$arg") ;;
    car_camera_info_file:=*) car_args+=("camera_info_file:=${arg#*:=}"); swarm_args+=("$arg") ;;
    mapping_config_file:=*|enable_icp:=*|enable_slam:=*|enable_calibration:=*|tf_topic:=*|tf_static_topic:=*|use_sim_time:=*) mapping_args+=("$arg") ;;
    anchor:=*|car_map_topic:=*|car_radius:=*|sensing:=*|separation:=*|max_position_variance:=*|max_yaw_variance:=*) swarm_args+=("$arg") ;;
    *) car_args+=("$arg") ;;
  esac
done

# Each enabled node starts once, including when a read-only mapping session is already running.
start_car() {
  local component node flag missing=0
  local components=(/robot_command:enable_command /robot_mission:enable_mission)
  if [[ $mode == mission ]]; then
    car_args=("config_file:=$(ros2 pkg prefix --share car_bringup)/config/bringup.yaml" "${car_args[@]}")
  else
    components+=(/ugv_bridge_node:enable_bridge /robot_video:enable_video /slam_toolbox:global_map_enabled
                 /car_camera_mount:enable_camera_mount /vision_node:enable_detector /vision_mission:enable_vision_mission)
  fi
  for component in "${components[@]}"; do
    node=${component%:*}; flag=${component#*:}
    if has_node "$node"; then car_args+=("$flag:=false"); else missing=1; fi
  done
  if ((missing)); then
    local prefix="bash \"$workspace/Scripts/bx72_run.sh\" --car-node"
    if [[ $mode == mission ]]; then start ros2 launch --launch-prefix "$prefix" car_navigation mission_services.launch.py "${car_args[@]}"
    else start ros2 launch --launch-prefix "$prefix" car_bringup car_bringup.launch.py "${car_args[@]}"; fi
  fi
}

calib="$project/swarm_ws/calibration"
mount="$calib/acg720_camera_mount_20261116.yaml"
acg_info="$calib/acg720_800x480_preliminary_20261116.yaml"
start_mapping() {
  local component missing=0
  for component in /acg720/icp_odometry:enable_icp /acg720/slam_toolbox:enable_slam /acg720/camera_calibration:enable_calibration; do
    if has_node "${component%:*}"; then mapping_args+=("${component#*:}:=false"); else missing=1; fi
  done
  if ((missing)); then
    start ros2 launch legged_bringup acg720_mapping.launch.py "camera_mount_file:=$mount" \
      "camera_info_file:=$acg_info" "${mapping_args[@]}"
  fi
}
case "$mode" in
  car|mission) start_car ;;
  mapping) start_mapping ;;
  all|cooperative)
    case ",${members// /}," in *,car,*) start_car ;; esac
    case ",${members// /}," in *,acg720,*) start_mapping ;; esac
    if ! has_node /swarm_coordinator; then
      start ros2 launch swarm_cooperation cooperative_mapping.launch.py start_car_mapping:=false start_acg720_mapping:=false \
        "car_camera_info_file:=$calib/car_ov5647_640x360_preliminary_1794469851452.yaml" \
        "acg_camera_info_file:=$acg_info" \
        "acg_camera_mount_file:=$mount" "${swarm_args[@]}"
    fi ;;
esac

dashboard_running() {
  local cmd port_arg='[[:space:]]--port(=|[[:space:]]+)([^[:space:]]+)'
  while IFS= read -r cmd; do
    if [[ ! $cmd =~ $port_arg || ${BASH_REMATCH[2]} == 8765 ]]; then return 0; fi
  done < <(pgrep -a -u "$UID" -f '(^|[ /])dashboard_server\.py([[:space:]]|$)')
  return 1
}
if [[ $mode != mapping ]] && ! dashboard_running; then
  if [[ $(systemctl --user show -p LoadState --value bx72-dashboard.service 2>/dev/null) == loaded ]]; then
    systemctl --user start bx72-dashboard.service
  else
    start python3 -u "$project/website/dashboard_server.py" --bind 0.0.0.0 --port 8765
  fi
fi
if ((${#children[@]} == 0)); then printf 'Requested nodes are already running.\n'; fi
# A launch whose nodes were disabled can exit successfully while its peers keep running.
while ((${#children[@]})); do
  for index in "${!children[@]}"; do
    pid=${children[index]}
    if kill -0 "$pid" 2>/dev/null; then continue; fi
    wait "$pid" || exit $?
    unset 'children[index]'
  done
  if ((${#children[@]})); then sleep 1; fi
done
