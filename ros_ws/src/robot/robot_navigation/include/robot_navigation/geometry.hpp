#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

namespace robot_navigation {

using Room = std::array<double, 4>;
using CircularObstacle = std::array<double, 3>;

// 返回射线与矩形房间或可选圆障碍物的最近交点距离。
inline double ray_distance(
  double x, double y, double angle, const Room & room,
  const std::optional<CircularObstacle> & obstacle = std::nullopt,
  double max_range = 8.0)
{
  const double direction_x = std::cos(angle);
  const double direction_y = std::sin(angle);
  double nearest = std::numeric_limits<double>::infinity();

  if (std::abs(direction_x) > 1e-12) {
    for (const double wall_x : {room[0], room[1]}) {
      const double distance = (wall_x - x) / direction_x;
      const double hit_y = y + distance * direction_y;
      if (distance > 0.0 && hit_y >= room[2] && hit_y <= room[3]) {
        nearest = std::min(nearest, distance);
      }
    }
  }
  if (std::abs(direction_y) > 1e-12) {
    for (const double wall_y : {room[2], room[3]}) {
      const double distance = (wall_y - y) / direction_y;
      const double hit_x = x + distance * direction_x;
      if (distance > 0.0 && hit_x >= room[0] && hit_x <= room[1]) {
        nearest = std::min(nearest, distance);
      }
    }
  }

  if (obstacle) {
    const double offset_x = x - (*obstacle)[0];
    const double offset_y = y - (*obstacle)[1];
    const double projection = offset_x * direction_x + offset_y * direction_y;
    const double radius = (*obstacle)[2];
    const double discriminant = projection * projection -
      (offset_x * offset_x + offset_y * offset_y - radius * radius);
    if (discriminant >= 0.0) {
      const double distance = -projection - std::sqrt(discriminant);
      if (distance > 0.0) {
        nearest = std::min(nearest, distance);
      }
    }
  }

  return std::isfinite(nearest) ? nearest : max_range;
}

}  // namespace robot_navigation
