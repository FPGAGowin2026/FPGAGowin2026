#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

namespace car_navigation {

struct Point {
  double x, y;
};
// Metres, matching the footprint and footprint_padding in waypoint_nav.yaml.
inline constexpr double body_padding = 0.020;
inline constexpr double body_half_length = 0.130 + body_padding;
inline constexpr double body_half_width = 0.125 + body_padding;
inline constexpr double intermediate_waypoint_radius = 0.10;
inline const double body_radius = std::hypot(body_half_length, body_half_width);
// Braking range at 0.50 m/s plus the lidar's forward offset, rounded to a map cell.
inline constexpr double obstacle_check_distance = 1.10;

struct Route {
  std::vector<Point> points;
  std::string reason;
};

// Centre and half-side are in destination cell units. A positive-area overlap
// must pass the separating-axis checks in both grids. Return true to stop early.
template <class Visit>
bool visit_overlapping_cells(Point center, double half_side, double c, double s, int width, int height,
                             Visit visit) {
  const double span = std::abs(c) + std::abs(s);
  const double extent = half_side * span;
  const double separation = half_side + 0.5 * span - 1e-9;
  if (center.x + extent <= 0 || center.y + extent <= 0 || center.x - extent >= width ||
      center.y - extent >= height)
    return false;
  const auto first = [&](double v, int limit) {
    return static_cast<int>(std::clamp(std::floor(v - extent + 1e-8), 0.0, double(limit)));
  };
  const auto end = [&](double v, int limit) {
    return static_cast<int>(std::clamp(std::ceil(v + extent - 1e-8), 0.0, double(limit)));
  };
  const int x0 = first(center.x, width), x1 = end(center.x, width);
  const int y0 = first(center.y, height), y1 = end(center.y, height);
  for (int y = y0; y < y1; ++y) {
    for (int x = x0; x < x1; ++x) {
      const double dx = x + 0.5 - center.x, dy = y + 0.5 - center.y;
      if (std::abs(c * dx + s * dy) < separation && std::abs(c * dy - s * dx) < separation && visit(x, y))
        return true;
    }
  }
  return false;
}

// The complete projected square must lie on known free cells of the destination grid.
bool known_free_square(const nav_msgs::msg::OccupancyGrid &map, Point center, double half,
                       double c, double s);

// Angles are in the map frame. The base translates at travel_yaw, with optional
// in-place turns at the beginning and end of the segment.
bool clear_body_segment(const nav_msgs::msg::OccupancyGrid &map, Point from, Point to, double travel_yaw,
                        bool allow_unknown, std::optional<Point> start_clear,
                        const nav_msgs::msg::OccupancyGrid *peer, std::optional<double> start_yaw = {},
                        std::optional<double> end_yaw = {});
// Use with a known-clear global sweep. The front lidar can leave unknown cells
// under the parked footprint, but every newly swept cell must be observed free.
bool clear_observed_departure(const nav_msgs::msg::OccupancyGrid &map, Point from, Point to, double yaw);
nav_msgs::msg::OccupancyGrid::ConstSharedPtr recovery_local_map(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &local, const nav_msgs::msg::OccupancyGrid &fused,
    Point odom, Point robot, double odom_yaw, double odom_to_map_yaw);
double frontier_heading(const nav_msgs::msg::OccupancyGrid &map, Point point, double fallback_yaw);

struct Waypoint {
  Point point;
  double yaw;
};
struct MissionRoute {
  Route route;
  bool known = false;
};

// blocked_target is the endpoint of the obstructed reference in the map frame.
Route local_detour(const nav_msgs::msg::OccupancyGrid &map, const nav_msgs::msg::OccupancyGrid &local,
                   Point start, Point odom, double map_to_odom_yaw, Point goal,
                   const std::vector<Point> &excluded, std::optional<Point> start_clear,
                   const std::function<bool()> &cancel, const nav_msgs::msg::OccupancyGrid *peer,
                   std::optional<double> start_yaw = {}, std::optional<double> goal_yaw = {},
                   std::optional<Point> blocked_target = {});
// Only advance a nearby corner when the bypass and its turn onto the next edge fit.
struct RouteCursor {
  std::size_t next;
  bool retained_corner;
};
RouteCursor tracking_vertex(const nav_msgs::msg::OccupancyGrid &map, const std::vector<Point> &points,
                            std::size_t next, Point robot, std::optional<double> yaw, bool allow_unknown,
                            std::optional<Point> start_clear, const nav_msgs::msg::OccupancyGrid *peer,
                            std::optional<double> end_yaw, bool can_strafe, std::optional<Point> previous = {});
bool repeats_blocked_leg(const nav_msgs::msg::OccupancyGrid &map, const std::vector<Point> &points,
                         std::size_t next, Point start, Point blocked_target, std::optional<double> yaw,
                         bool allow_unknown, std::optional<Point> start_clear,
                         const nav_msgs::msg::OccupancyGrid *peer, std::optional<double> end_yaw, bool can_strafe);

double distance(Point a, Point b);
double segment_distance(Point p, Point a, Point b);
MissionRoute build_route(const nav_msgs::msg::OccupancyGrid &map, Point start, double yaw,
                         std::optional<Point> goal, std::optional<double> goal_yaw, bool use_mppi,
                         std::optional<Point> start_clear, const std::vector<Point> &excluded,
                         const std::function<bool()> &cancel,
                         const nav_msgs::msg::OccupancyGrid *peer = nullptr);
nav_msgs::msg::Path controller_path(const std::vector<Point> &points, double final_yaw,
                                    const std_msgs::msg::Header &header, double resolution);
bool clear_ahead(const nav_msgs::msg::OccupancyGrid &map, const std::vector<Point> &points, std::size_t next,
                 Point robot, bool allow_unknown, std::optional<Point> start_clear,
                 const nav_msgs::msg::OccupancyGrid *peer = nullptr, std::optional<double> start_yaw = {},
                 std::optional<double> end_yaw = {}, std::optional<double> held_yaw = {});
}  // namespace car_navigation
