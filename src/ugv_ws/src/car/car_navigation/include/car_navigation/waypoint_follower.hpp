#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace car_navigation {
class WaypointFollower {
 public:
  explicit WaypointFollower(rclcpp::Node& node);
  void start_or_update(const nav_msgs::msg::Path& path);
  void cancel();
  void tick(double front_range);
  bool server_ready() const;
  std::string state() const;
  bool recently_following(bool allow_aborted) const;

 private:
  using Action = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using Steady = std::chrono::steady_clock;

  struct OutstandingGoal {
    GoalHandle::SharedPtr handle;
    Steady::time_point accept_until;
    bool cancel_sent{};
  };

  void dispatch();
  void poll_stack();
  void cancel_handle(std::uint64_t generation, const GoalHandle::SharedPtr& handle);
  void on_velocity(const geometry_msgs::msg::Twist& twist, const rclcpp::MessageInfo& info);

  rclcpp::Node& node_;
  mutable std::mutex mutex_;
  nav_msgs::msg::Path path_;
  std::uint64_t generation_ = 0, cancel_through_ = 0;
  std::string state_ = "idle";
  bool permitted_ = false;
  bool tracking_ = false;
  bool path_changed_ = false;
  double front_range_{};
  Steady::time_point permit_until_{};
  Steady::time_point request_since_{}, stack_active_until_{};
  std::int64_t accept_velocity_after_ns_ = 0;
  std::optional<Steady::time_point> feedback_at_;
  std::map<std::uint64_t, OutstandingGoal> outstanding_;

  rclcpp::CallbackGroup::SharedPtr controller_group_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  using StackStatus = std_srvs::srv::Trigger;
  rclcpp::Client<StackStatus>::SharedPtr stack_status_;
  std::optional<std::int64_t> stack_request_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_sub_, velocity_sub_;
  rclcpp::TimerBase::SharedPtr timer_, stack_timer_;
};
}  // namespace car_navigation
