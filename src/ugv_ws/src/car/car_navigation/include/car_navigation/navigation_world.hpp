#pragma once

#include "car_navigation/grid_route.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

namespace car_navigation {
inline constexpr float front_stop_range = 0.15F;  // LaserScan ranges are metres from the lidar.

struct FrontScan {
  std::vector<Point> hits;  // odom at scan time
  Point mount;
  double mount_yaw{}, half_angle{};
  bool blocks_translation(Point from, Point to, double yaw) const;
  bool blocks_turn(Point center, double from_yaw, double to_yaw) const;
};

struct WorldSnapshot {
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map, observation_map, local_map, peer_map;
  std::shared_ptr<const FrontScan> front_scan;
  std::optional<std::uint32_t> fusion_nav_status;
  bool localized{}, wheel_trusted{}, scan_fresh{}, front_stop_held{}, planning_local_map{}, planning_laser_clear{};
  double front_range{};
  bool peer_map_ready{};
  double x{}, y{}, yaw{}, odom_x{}, odom_y{}, odom_yaw{}, vx{}, vy{}, wz{};
  double laser_age_s{-1.0};
  std::uint64_t generation{}, correction_revision{}, map_version{}, base_sequence{};
  std::string map_session, localization_state{"waiting"}, localization_reason{"waiting_for_laser"};
  unsigned confirmations{};
};

class NavigationWorld {
 public:
  explicit NavigationWorld(rclcpp::Node& node);
  ~NavigationWorld();
  WorldSnapshot snapshot() const;
  void set_cooperative(bool enabled);
  void set_fpga_goal_speed_mm_s(unsigned speed);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace car_navigation
