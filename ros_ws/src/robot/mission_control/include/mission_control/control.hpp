#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace mission_control {

struct ApproachCommand {
  double linear;
  double angular;
  bool complete;
};

// 把归一化图像偏差转换为差速底盘接近命令。
inline ApproachCommand approach_command(
  double horizontal_error, double area_fraction, double stop_area,
  double alignment_tolerance, double forward_speed, double angular_gain,
  double max_angular_speed)
{
  const std::array<double, 7> values{
    horizontal_error, area_fraction, stop_area, alignment_tolerance,
    forward_speed, angular_gain, max_angular_speed,
  };
  if (!std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    }) || area_fraction < 0.0)
  {
    return {0.0, 0.0, false};
  }
  if (area_fraction >= stop_area) {
    return {0.0, 0.0, true};
  }

  const double angular = std::clamp(
    -angular_gain * horizontal_error, -max_angular_speed, max_angular_speed);
  const double linear = std::abs(horizontal_error) <= alignment_tolerance ?
    forward_speed : 0.0;
  return {linear, angular, false};
}

}  // namespace mission_control
