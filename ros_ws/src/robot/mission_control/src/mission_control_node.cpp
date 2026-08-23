#include "mission_control/control.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mission_control {
namespace {

constexpr const char * kWaitingNav = "WAITING_NAV";
constexpr const char * kNavigating = "NAVIGATING";
constexpr const char * kSearching = "SEARCHING";
constexpr const char * kTracking = "TRACKING";
constexpr const char * kComplete = "COMPLETE";
constexpr const char * kFailed = "FAILED";
constexpr const char * kStopped = "STOPPED";
constexpr const char * kIdle = "IDLE";

using SteadyClock = std::chrono::steady_clock;

struct Target {
  double horizontal_error;
  double area_fraction;
};

}  // namespace

class MissionControlNode : public rclcpp::Node {
 public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MissionControlNode()
  : Node("mission_control_node")
  {
    const bool autostart = declare_parameter<bool>("autostart", false);
    const auto command_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_nav");
    search_goal_x_ = declare_parameter<double>("search_goal_x_m", 1.0);
    search_goal_y_ = declare_parameter<double>("search_goal_y_m", 0.0);
    search_goal_yaw_ = declare_parameter<double>("search_goal_yaw_rad", 0.0);
    search_speed_ = declare_parameter<double>("search_angular_speed_rad_s", 0.35);
    approach_speed_ = declare_parameter<double>("approach_speed_m_s", 0.12);
    alignment_tolerance_ = declare_parameter<double>("alignment_tolerance", 0.12);
    angular_gain_ = declare_parameter<double>("angular_gain", 1.2);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed_rad_s", 0.6);
    stop_area_ = declare_parameter<double>("target_stop_area_fraction", 0.06);
    target_timeout_ = declare_parameter<double>("target_timeout_s", 0.5);
    search_timeout_ = declare_parameter<double>("search_timeout_s", 60.0);
    navigation_start_timeout_ = declare_parameter<double>(
      "navigation_start_timeout_s", 30.0);
    if (search_speed_ <= 0.0 || approach_speed_ <= 0.0 || alignment_tolerance_ <= 0.0 ||
      angular_gain_ <= 0.0 || max_angular_speed_ <= 0.0 || stop_area_ <= 0.0 ||
      target_timeout_ <= 0.0 || search_timeout_ <= 0.0 || navigation_start_timeout_ <= 0.0)
    {
      throw std::invalid_argument("任务速度、阈值与超时参数必须为正数");
    }

    // Nav2 控制器和视觉接近共用平滑器入口，最终只有速度平滑器发布 /cmd_vel。
    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(command_topic, 10);
    state_publisher_ = create_publisher<std_msgs::msg::String>(
      "/mission/state", rclcpp::QoS(1).transient_local());
    target_subscription_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/vision/target_observation", 10,
      std::bind(&MissionControlNode::on_target, this, std::placeholders::_1));
    estop_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop", 10,
      std::bind(&MissionControlNode::on_emergency_stop, this, std::placeholders::_1));
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "/mission/start",
      std::bind(
        &MissionControlNode::on_start, this,
        std::placeholders::_1, std::placeholders::_2));
    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&MissionControlNode::update, this));

    publish_state(kIdle);
    if (autostart) {
      begin_mission();
    }
  }

  void stop()
  {
    publish_command();
  }

 private:
  void publish_state(const std::string & state)
  {
    if (state == state_) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "任务状态: %s -> %s",
      state_.empty() ? "START" : state_.c_str(), state.c_str());
    state_ = state;
    std_msgs::msg::String message;
    message.data = state;
    state_publisher_->publish(message);
  }

  void publish_command(double linear = 0.0, double angular = 0.0)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = linear;
    command.angular.z = angular;
    command_publisher_->publish(command);
  }

  void on_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (emergency_stop_) {
      response->success = false;
      response->message = "急停有效，拒绝启动任务";
      return;
    }
    if (state_ == kWaitingNav || state_ == kNavigating ||
      state_ == kSearching || state_ == kTracking)
    {
      response->success = false;
      response->message = "任务正在运行: " + state_;
      return;
    }
    begin_mission();
    response->success = true;
    response->message = "任务已启动";
  }

  void begin_mission()
  {
    goal_handle_.reset();
    goal_request_pending_ = false;
    target_.reset();
    const auto now = SteadyClock::now();
    navigation_deadline_ = now + std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(navigation_start_timeout_));
    next_goal_attempt_ = SteadyClock::time_point::min();
    publish_command();
    publish_state(kWaitingNav);
  }

  void on_target(const geometry_msgs::msg::Vector3Stamped::SharedPtr message)
  {
    target_ = Target{message->vector.x, message->vector.z};
    target_time_ = SteadyClock::now();
  }

  void on_emergency_stop(const std_msgs::msg::Bool::SharedPtr message)
  {
    emergency_stop_ = message->data;
    if (!emergency_stop_) {
      return;
    }
    if (goal_handle_) {
      navigation_client_->async_cancel_goal(goal_handle_);
    }
    goal_request_pending_ = false;
    publish_command();
    publish_state(kStopped);
  }

  void update()
  {
    if (emergency_stop_) {
      publish_command();
      return;
    }
    if (state_ == kWaitingNav) {
      if (SteadyClock::now() > navigation_deadline_) {
        publish_state(kFailed);
        return;
      }
      try_send_navigation_goal();
      return;
    }
    if (state_ != kSearching && state_ != kTracking) {
      return;
    }

    const auto now = SteadyClock::now();
    if (std::chrono::duration<double>(now - search_start_time_).count() > search_timeout_) {
      publish_command();
      publish_state(kFailed);
      return;
    }
    if (!target_ ||
      std::chrono::duration<double>(now - target_time_).count() > target_timeout_)
    {
      publish_state(kSearching);
      publish_command(0.0, search_speed_);
      return;
    }

    publish_state(kTracking);
    const auto command = approach_command(
      target_->horizontal_error, target_->area_fraction, stop_area_, alignment_tolerance_,
      approach_speed_, angular_gain_, max_angular_speed_);
    publish_command(command.linear, command.angular);
    if (command.complete) {
      publish_command();
      publish_state(kComplete);
    }
  }

  void try_send_navigation_goal()
  {
    const auto now = SteadyClock::now();
    if (goal_request_pending_ || now < next_goal_attempt_ ||
      !navigation_client_->action_server_is_ready())
    {
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = get_clock()->now();
    goal.pose.pose.position.x = search_goal_x_;
    goal.pose.pose.position.y = search_goal_y_;
    goal.pose.pose.orientation.z = std::sin(search_goal_yaw_ * 0.5);
    goal.pose.pose.orientation.w = std::cos(search_goal_yaw_ * 0.5);
    goal_request_pending_ = true;
    next_goal_attempt_ = now + std::chrono::seconds(1);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = std::bind(
      &MissionControlNode::on_goal_response, this, std::placeholders::_1);
    options.result_callback = std::bind(
      &MissionControlNode::on_navigation_result, this, std::placeholders::_1);
    navigation_client_->async_send_goal(goal, options);
  }

  void on_goal_response(const GoalHandle::SharedPtr & goal_handle)
  {
    goal_request_pending_ = false;
    if (emergency_stop_) {
      if (goal_handle) {
        navigation_client_->async_cancel_goal(goal_handle);
      }
      return;
    }
    if (!goal_handle) {
      // Action 服务会早于 Nav2 生命周期激活；启动窗口内拒绝目标时等待后重试。
      if (SteadyClock::now() < navigation_deadline_) {
        publish_state(kWaitingNav);
      } else {
        publish_command();
        publish_state(kFailed);
      }
      return;
    }
    goal_handle_ = goal_handle;
    publish_state(kNavigating);
  }

  void on_navigation_result(const GoalHandle::WrappedResult & result)
  {
    goal_handle_.reset();
    if (emergency_stop_) {
      return;
    }
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      if (SteadyClock::now() < navigation_deadline_) {
        publish_state(kWaitingNav);
      } else {
        publish_command();
        publish_state(kFailed);
      }
      return;
    }

    // Nav2 已结束并停车后才切换到直接视觉接近，任一时刻只有一个运动命令源。
    target_.reset();
    search_start_time_ = SteadyClock::now();
    publish_state(kSearching);
  }

  double search_goal_x_{};
  double search_goal_y_{};
  double search_goal_yaw_{};
  double search_speed_{};
  double approach_speed_{};
  double alignment_tolerance_{};
  double angular_gain_{};
  double max_angular_speed_{};
  double stop_area_{};
  double target_timeout_{};
  double search_timeout_{};
  double navigation_start_timeout_{};
  std::string state_;
  bool emergency_stop_{false};
  bool goal_request_pending_{false};
  std::optional<Target> target_;
  SteadyClock::time_point target_time_{};
  SteadyClock::time_point search_start_time_{};
  SteadyClock::time_point navigation_deadline_{};
  SteadyClock::time_point next_goal_attempt_{};
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr target_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mission_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<mission_control::MissionControlNode>();
    rclcpp::spin(node);
    if (rclcpp::ok()) {
      node->stop();
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mission_control_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
