#pragma once

#include "car_navigation/grid_route.hpp"
#include "car_navigation/navigation_world.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nlohmann/json.hpp>

namespace car_navigation {
// Owned by the mission node's mutually-exclusive callback group.
class FpgaReference {
 public:
  explicit FpgaReference(rclcpp::Node& node);
  void reset();
  void on_command(const nlohmann::json& status);
  bool temporary_start_active(const nlohmann::json& telemetry);
  bool arrived_at(const nlohmann::json& telemetry, const std::string& kind, Point endpoint) const;
  bool active() const { return sent_.has_value(); }
  bool blocked() const { return blocked_; }
  bool needs_replan() const { return needs_replan_; }
  const std::string& block_reason() const { return block_reason_; }
  std::optional<Point> excluded_frontier() const { return excluded_frontier_; }
  // Fixed odom target of a translation obstructed on the board or in the scan preview.
  std::optional<Point> blocked_target() const;
  bool scan_blocked() const { return scan_block_.has_value(); }
  void update(const WorldSnapshot& world, const std::vector<Point>& remaining, const std::string& kind,
              bool final_waypoint, double goal_yaw, const nlohmann::json& telemetry,
              std::optional<geometry_msgs::msg::PoseStamped> start_odom, bool retained_corner = false,
              std::optional<Point> intermediate_waypoint = {});

 private:
  using Pose = geometry_msgs::msg::PoseStamped;
  using Path = nav_msgs::msg::Path;
  using Clock = std::chrono::steady_clock;
  enum class Role { segment, corner, approach, goal, frontier, align };
  struct Target {
    Point map;
    double yaw;
    Role role;
    bool explore;
    bool pass_point{};
  };
  struct Submission {
    Target target;
    Point odom;
    double odom_yaw;
    std::int64_t stamp;
    Clock::time_point sent_at;
    std::uint32_t task, boot, domain;
    std::uint64_t token;
    double hold_odom_yaw;
    std::string route_kind;
    Point route_endpoint;
  };
  bool matches(const nlohmann::json& telemetry) const;
  void clear_blockage();
  bool observe_blockage(const WorldSnapshot& world, const nlohmann::json& telemetry);
  rclcpp::Node& node_;
  rclcpp::Publisher<Pose>::SharedPtr goal_, reference_;
  rclcpp::Publisher<Path>::SharedPtr confirmed_goal_, confirmed_reference_, explore_reference_, corner_reference_;
  std::optional<Submission> sent_;
  // The board's temporary footprint survives cancellation until its original deadline.
  std::int64_t last_start_stamp_{};
  Clock::time_point possible_start_until_{};
  std::uint32_t start_boot_{}, start_domain_{}, reported_boot_{}, reported_domain_{};
  std::optional<std::uint32_t> board_start_until_;
  bool blocked_ = false;
  struct Blockage {
    std::uint32_t since_us;
    Point position;
    double yaw;
    bool wheel_trusted;
    bool obstacle;
  };
  std::optional<Blockage> blockage_;
  std::optional<Point> scan_block_;
  bool needs_replan_ = false;
  std::string block_reason_;
  std::optional<Point> excluded_frontier_;
};
}  // namespace car_navigation
