#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "错误：请使用 source Scripts/setup_ros_dependencies.bash" >&2
  exit 1
fi

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "错误：请先执行 source /opt/ros/humble/setup.bash" >&2
  return 1
fi

_project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
_dependency_root="${_project_root}/ros_ws/src/external/ros_humble_overlay"
_ros_prefix="${_dependency_root}/opt/ros/humble"

if [[ ! -f "${_ros_prefix}/share/ament_index/resource_index/packages/nav2_bringup" ]]; then
  echo "错误：本地 ROS 依赖未安装，请先运行 ./Scripts/install_ros_dependencies.sh" >&2
  return 1
fi

_prepend_ros_dependency_path() {
  local variable_name="$1"
  local path="$2"
  local current_value="${!variable_name-}"

  [[ -d "${path}" ]] || return 0
  case ":${current_value}:" in
    *":${path}:"*) return 0 ;;
  esac
  printf -v "${variable_name}" '%s' "${path}${current_value:+:${current_value}}"
  export "${variable_name}"
}

_prepend_ros_dependency_path AMENT_PREFIX_PATH "${_ros_prefix}"
_prepend_ros_dependency_path CMAKE_PREFIX_PATH "${_ros_prefix}"
_prepend_ros_dependency_path LD_LIBRARY_PATH "${_dependency_root}/usr/lib/x86_64-linux-gnu"
_prepend_ros_dependency_path LD_LIBRARY_PATH "${_dependency_root}/usr/lib"
_prepend_ros_dependency_path LD_LIBRARY_PATH "${_ros_prefix}/lib"
_prepend_ros_dependency_path PYTHONPATH "${_ros_prefix}/lib/python3.10/site-packages"
_prepend_ros_dependency_path PYTHONPATH "${_ros_prefix}/local/lib/python3.10/dist-packages"
_prepend_ros_dependency_path PATH "${_ros_prefix}/bin"

unset _dependency_root _project_root _ros_prefix
unset -f _prepend_ros_dependency_path
