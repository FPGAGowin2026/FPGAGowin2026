#include "car_navigation/waypoint_follower.hpp"
#include "car_navigation/navigation_world.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace car_navigation {
namespace {
using namespace std::chrono_literals;
constexpr auto velocity_lease = 150ms;
constexpr auto permission_lease = 250ms;
constexpr auto feedback_lease = 750ms;
constexpr auto server_response_timeout = 2s;
constexpr double max_forward = 0.50;

double front_speed_scale(double vx, double wz, double range) {
  // BX72 trial profile (robot_config.h and publish_front_threshold). The board
  // also checks measured motion and retained hits before enabling the wheels.
  constexpr double radius = 0.181;
  constexpr double brake_x = 0.20, brake_yaw = 1.50, delay = 0.20;
  const double budget = range - front_stop_range;
  if (budget <= 0.) return 0.;
  const double latency = delay * (vx + radius * std::abs(wz));
  const double braking = vx * vx / (2 * brake_x) + radius * wz * wz / (2 * brake_yaw);
  if (latency + braking <= budget) return 1.;
  // Solve braking * scale^2 + latency * scale = budget for both axes together.
  return 2 * budget / (latency + std::sqrt(latency * latency + 4 * braking * budget));
}

std::int64_t system_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

bool same_path(const nav_msgs::msg::Path &a, const nav_msgs::msg::Path &b) {
  if (a.header.frame_id != b.header.frame_id || a.poses.size() != b.poses.size()) return false;
  return std::equal(a.poses.begin(), a.poses.end(), b.poses.begin(), [](const auto &x, const auto &y) {
    return x.header.frame_id == y.header.frame_id && x.pose == y.pose;
  });
}
}  // namespace

WaypointFollower::WaypointFollower(rclcpp::Node &node) : node_(node) {
  controller_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_ = rclcpp_action::create_client<Action>(&node_, "/waypoint_nav/follow_path", controller_group_);
  stack_status_ = node_.create_client<StackStatus>(
      "/waypoint_nav/lifecycle_manager/is_active", rmw_qos_profile_services_default, controller_group_);
  stack_timer_ = node_.create_wall_timer(500ms, [this] { poll_stack(); }, controller_group_);
  velocity_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>("/fpga/waypoint_velocity", 1);
  rclcpp::SubscriptionOptions options;
  options.callback_group = controller_group_;
  drive_pub_ = node_.create_publisher<geometry_msgs::msg::Twist>("/waypoint_nav/cmd_vel_drive", 1);
  raw_sub_ = node_.create_subscription<geometry_msgs::msg::Twist>(
      "/waypoint_nav/cmd_vel_raw", 1,
      [this](const geometry_msgs::msg::Twist &input, const rclcpp::MessageInfo &info) {
        const auto age = std::chrono::nanoseconds(system_ns() - info.get_rmw_message_info().source_timestamp);
        if (age.count() < 0 || age >= velocity_lease) return;
        // MPPI smooths after clamping vx. Check the actual pure turn against
        // the footprint when smoothing undershoots zero within the odometry deadband.
        auto cmd = input;
        if (cmd.linear.x < 0.) {
          if (cmd.linear.x >= -0.005) cmd.linear.x = 0.;
          else cmd = geometry_msgs::msg::Twist{};
        }
        drive_pub_->publish(cmd);
      },
      options);
  velocity_sub_ = node_.create_subscription<geometry_msgs::msg::Twist>(
      "/waypoint_nav/cmd_vel_safe", 1,
      [this](const geometry_msgs::msg::Twist &twist, const rclcpp::MessageInfo &info) {
        on_velocity(twist, info);
      },
      options);
  timer_ = node_.create_wall_timer(std::chrono::milliseconds(50), [this] { dispatch(); }, controller_group_);
}

void WaypointFollower::start_or_update(const nav_msgs::msg::Path &path) {
  if (path.poses.empty()) {
    cancel();
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != "failed" && same_path(path_, path)) return;
  path_ = path;
  path_changed_ = true;
  if (state_ == "idle" || state_ == "failed" || state_ == "succeeded") {
    request_since_ = Steady::now();
    state_ = "waiting_server";
  }
}

void WaypointFollower::cancel() {
  std::lock_guard<std::mutex> lock(mutex_);
  cancel_through_ = generation_;
  path_.poses.clear();
  state_ = "idle";
  path_changed_ = false;
  permitted_ = tracking_ = false;
  feedback_at_.reset();
}

void WaypointFollower::tick(double front_range) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = Steady::now();
  front_range_ = front_range;
  if (!permitted_ || now >= permit_until_) {
    accept_velocity_after_ns_ = system_ns();
    feedback_at_.reset();
  }
  permit_until_ = now + permission_lease;
  permitted_ = true;
}

void WaypointFollower::poll_stack() {
  // The lifecycle manager confirms both the controller and collision monitor.
  // The timer and reply share a mutually exclusive callback group.
  if (stack_request_) {
    stack_status_->remove_pending_request(*stack_request_);
    stack_request_.reset();
  }
  if (!stack_status_->service_is_ready()) return;
  const auto sent = Steady::now();
  try {
    stack_request_ = stack_status_->async_send_request(
        std::make_shared<StackStatus::Request>(),
        [this, sent](rclcpp::Client<StackStatus>::SharedFuture response) {
          std::lock_guard<std::mutex> lock(mutex_);
          stack_active_until_ = response.get()->success ? sent + 1500ms : Steady::time_point{};
          stack_request_.reset();
        }).request_id;
  } catch (const rclcpp::exceptions::RCLError &error) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "Nav2 status request failed: %s", error.what());
  }
}

bool WaypointFollower::server_ready() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return Steady::now() < stack_active_until_ && stack_status_->service_is_ready() && client_->action_server_is_ready();
}

std::string WaypointFollower::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = Steady::now();
  const bool waiting = state_ == "waiting_server" || state_ == "pending";
  const bool needs_response = waiting || (state_ == "active" && !tracking_);
  if (((waiting || state_ == "active") && now >= stack_active_until_) ||
      (needs_response && now - request_since_ >= server_response_timeout))
    return "unavailable";
  return state_;
}

bool WaypointFollower::recently_following(bool allow_aborted) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = Steady::now();
  return permitted_ && now < permit_until_ && feedback_at_ && now - *feedback_at_ < feedback_lease &&
         (state_ == "active" || (allow_aborted && state_ == "failed"));
}

void WaypointFollower::cancel_handle(std::uint64_t generation, const GoalHandle::SharedPtr &handle) {
  try {
    client_->async_cancel_goal(handle, [this, generation](auto response) {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto it = outstanding_.find(generation);
      if (it == outstanding_.end()) return;
      if (response->return_code == response->ERROR_UNKNOWN_GOAL_ID ||
          response->return_code == response->ERROR_GOAL_TERMINATED)
        outstanding_.erase(it);
      else if (response->return_code == response->ERROR_REJECTED)
        it->second.cancel_sent = false;
    });
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
    std::lock_guard<std::mutex> lock(mutex_);
    outstanding_.erase(generation);
  } catch (const rclcpp::exceptions::RCLError &error) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = outstanding_.find(generation);
    if (it != outstanding_.end()) it->second.cancel_sent = false;
    RCLCPP_WARN(node_.get_logger(), "FollowPath cancel failed: %s", error.what());
  }
}

void WaypointFollower::dispatch() {
  Action::Goal goal;
  std::vector<std::pair<std::uint64_t, GoalHandle::SharedPtr>> retired;
  std::uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (outstanding_.empty() && !path_changed_) return;
    if (!client_->action_server_is_ready()) {
      if (!outstanding_.empty()) {
        cancel_through_ = generation_;
        outstanding_.clear();
        tracking_ = false;
        feedback_at_.reset();
        path_changed_ = !path_.poses.empty();
        state_ = path_changed_ ? "unavailable" : "idle";
      }
      return;
    }
    const auto now = Steady::now();
    for (auto it = outstanding_.begin(); it != outstanding_.end() && it->first <= cancel_through_;) {
      auto &entry = it->second;
      if (!entry.handle && now >= entry.accept_until) {
        it = outstanding_.erase(it);
        continue;
      }
      if (entry.handle && !entry.cancel_sent) {
        retired.emplace_back(it->first, entry.handle);
        entry.cancel_sent = true;
      }
      ++it;
    }
    const bool cancelling = !outstanding_.empty() && outstanding_.begin()->first <= cancel_through_;
    // Humble's controller cancels both the active and pending goals on explicit stop.
    // Ordinary path updates preempt after the preceding goal starts its controller cycle.
    if (path_changed_ && !cancelling && (state_ == "waiting_server" || (state_ == "active" && tracking_)) &&
        permitted_ && now < permit_until_ && now < stack_active_until_) {
      generation = ++generation_;
      outstanding_.emplace(generation, OutstandingGoal{nullptr, now + server_response_timeout});
      path_changed_ = tracking_ = false;
      feedback_at_.reset();
      goal.path = path_;
      goal.controller_id = "FollowPath";
      goal.goal_checker_id = "waypoint_goal";
      state_ = "pending";
      request_since_ = now;
    }
  }
  for (const auto &[id, handle] : retired) cancel_handle(id, handle);
  if (goal.path.poses.empty()) return;

  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [this, generation](GoalHandle::SharedPtr accepted) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = outstanding_.find(generation);
    if (it == outstanding_.end()) {
      // A late acceptance can be cancelled while the stream is retired. Once
      // a new owner sends its goal, let Nav2 preempt: a late cancel can stop both.
      if (accepted && generation <= cancel_through_ && generation_ == cancel_through_)
        outstanding_.emplace(generation, OutstandingGoal{accepted, {}});
      return;
    }
    if (accepted)
      it->second.handle = accepted;
    else
      outstanding_.erase(it);
    if (generation == generation_ && generation > cancel_through_) {
      if (accepted) request_since_ = Steady::now();  // The first feedback confirms the controller cycle.
      else cancel_through_ = generation;
      state_ = accepted ? "active" : "unavailable";
    }
  };
  options.feedback_callback = [this, generation](GoalHandle::SharedPtr,
                                                 const std::shared_ptr<const Action::Feedback>) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != generation_ || generation <= cancel_through_ || state_ != "active") return;
    feedback_at_ = Steady::now();
    if (tracking_) return;
    // Acceptance of a preempting goal can precede its first controller cycle.
    outstanding_.erase(outstanding_.begin(), outstanding_.lower_bound(generation));
    accept_velocity_after_ns_ = system_ns();
    tracking_ = true;
  };
  options.result_callback = [this, generation](const GoalHandle::WrappedResult &result) {
    std::lock_guard<std::mutex> lock(mutex_);
    outstanding_.erase(generation);
    if (generation != generation_ || generation <= cancel_through_) return;
    cancel_through_ = generation;
    if (path_changed_) request_since_ = Steady::now();
    state_ = path_changed_                                         ? "waiting_server"
             : result.code == rclcpp_action::ResultCode::SUCCEEDED ? "succeeded"
                                                                   : "failed";
    tracking_ = false;
    // Humble can abort on controller patience while a blocked-leg solve is finishing.
    if (result.code != rclcpp_action::ResultCode::ABORTED || path_changed_) feedback_at_.reset();
  };
  try {
    client_->async_send_goal(goal, options);
  } catch (const rclcpp::exceptions::RCLError &error) {
    std::lock_guard<std::mutex> lock(mutex_);
    outstanding_.erase(generation);
    if (generation == generation_ && generation > cancel_through_) {
      cancel_through_ = generation;
      state_ = "unavailable";
    }
    RCLCPP_WARN(node_.get_logger(), "FollowPath request failed: %s", error.what());
  }
}

void WaypointFollower::on_velocity(const geometry_msgs::msg::Twist &twist, const rclcpp::MessageInfo &info) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = Steady::now();
  if (!permitted_ || now >= permit_until_ || now >= stack_active_until_ || !tracking_ || state_ != "active") return;
  // Humble Collision Monitor publishes unstamped Twist. The Fast DDS writer
  // timestamp preserves the original 150 ms lease across executor delays.
  const auto source = info.get_rmw_message_info().source_timestamp;
  if (source <= accept_velocity_after_ns_) return;
  const auto age = std::chrono::nanoseconds(system_ns() - source);
  if (age.count() < 0 || age >= velocity_lease) return;
  if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.angular.z) || twist.linear.x < 0.0 ||
      twist.linear.y != 0.0)
    return;

  geometry_msgs::msg::TwistStamped output;
  output.header.frame_id = "base_link";
  output.header.stamp = node_.now() - rclcpp::Duration(age);
  const double scale = std::max({1.0, twist.linear.x / max_forward, std::abs(twist.angular.z) / 0.80});
  const double vx = twist.linear.x / scale, wz = twist.angular.z / scale;
  const double near_obstacle = front_speed_scale(vx, wz, front_range_);
  output.twist.linear.x = vx * near_obstacle;
  output.twist.angular.z = wz * near_obstacle;
  accept_velocity_after_ns_ = source;
  velocity_pub_->publish(output);
}
}  // namespace car_navigation
