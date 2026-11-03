#include "uav_mission/mission_node.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>

namespace uav_mission
{

MissionNode::MissionNode()
: Node("uav_mission")
{
  FlightTaskConfig config;
#ifdef UAV_RELAY_ENABLED
  config.relay_mode = declare_parameter<bool>("relay_mode", true);
#else
  config.relay_mode = declare_parameter<bool>("relay_mode", false);
  if (config.relay_mode) {
    throw std::invalid_argument("Relay mode requires relay_mission_node and its optional dependencies");
  }
#endif
  config.takeoff_height_m = declare_parameter<double>("takeoff_height_m", 1.0);
  config.horizontal_tolerance_m = declare_parameter<double>("horizontal_tolerance_m", 0.2);
  config.vertical_tolerance_m = declare_parameter<double>("vertical_tolerance_m", 0.15);
  config.stable_duration_s = declare_parameter<double>("stable_duration_s", 2.0);
  config.hover_duration_s = declare_parameter<double>("hover_duration_s", 10.0);
  config.ready_timeout_s = declare_parameter<double>("ready_timeout_s", 30.0);
  config.command_timeout_s = declare_parameter<double>("command_timeout_s", 5.0);
  config.takeoff_timeout_s = declare_parameter<double>("takeoff_timeout_s", 30.0);
  config.landing_timeout_s = declare_parameter<double>("landing_timeout_s", 45.0);
  state_timeout_s_ = declare_parameter<double>("state_timeout_s", 1.0);
  if (!std::isfinite(config.takeoff_height_m) || config.takeoff_height_m <= 0.0 ||
    !std::isfinite(config.horizontal_tolerance_m) || config.horizontal_tolerance_m <= 0.0 ||
    !std::isfinite(config.vertical_tolerance_m) || config.vertical_tolerance_m <= 0.0 ||
    !std::isfinite(config.stable_duration_s) || config.stable_duration_s <= 0.0 ||
    !std::isfinite(config.hover_duration_s) || config.hover_duration_s <= 0.0 ||
    !std::isfinite(config.ready_timeout_s) || config.ready_timeout_s <= 0.0 ||
    !std::isfinite(config.command_timeout_s) || config.command_timeout_s <= 0.0 ||
    !std::isfinite(config.takeoff_timeout_s) || config.takeoff_timeout_s <= 0.0 ||
    !std::isfinite(config.landing_timeout_s) || config.landing_timeout_s <= 0.0 ||
    !std::isfinite(state_timeout_s_) || state_timeout_s_ <= 0.0)
  {
    throw std::invalid_argument("Mission heights, tolerances, and timeouts must be positive");
  }

  task_ = std::make_unique<FlightTask>(config);
#ifdef UAV_RELAY_ENABLED
  if (config.relay_mode) {
    relay_ = std::make_unique<RelayNavigation>(*this);
  }
#endif
  state_sub_ = create_subscription<uav_interfaces::msg::VehicleState>(
    "/uav_control/state", 10,
    std::bind(&MissionNode::state_callback, this, std::placeholders::_1));
  target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/uav_control/target", 10);
  command_client_ = create_client<uav_interfaces::srv::FlightCommand>(
    "/uav_control/command");
  cancel_service_ = create_service<std_srvs::srv::Trigger>(
    "/uav_mission/cancel",
    std::bind(
      &MissionNode::cancel_callback, this,
      std::placeholders::_1, std::placeholders::_2));
  timer_ = create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&MissionNode::timer_callback, this));

  RCLCPP_INFO(
    get_logger(), "Mission started: climb %.2f m, stable %.1f s, hover %.1f s",
    config.takeoff_height_m, config.stable_duration_s, config.hover_duration_s);
}

void MissionNode::state_callback(const uav_interfaces::msg::VehicleState::SharedPtr msg)
{
  latest_state_ = *msg;
  state_rx_ = Clock::now();
}

void MissionNode::cancel_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
#ifdef UAV_RELAY_ENABLED
  if (relay_) relay_->cancel("任务取消并请求降落");
#endif
  task_->cancel(Clock::now());
  response->success = true;
  response->message = "Cancellation accepted; task will land and confirm ground/disarmed state";
}

void MissionNode::timer_callback()
{
  const auto now_time = Clock::now();
  const bool stream_fresh = latest_state_ &&
    std::chrono::duration<double>(now_time - state_rx_).count() <= state_timeout_s_;
  const auto * state = latest_state_ ? &*latest_state_ : nullptr;
  auto result = task_->update(state, stream_fresh, now_time);
#ifdef UAV_RELAY_ENABLED
  if (relay_) relay_->tick(task_->stage_name(), state, stream_fresh, result);
#endif

  if (task_->stage_name() != last_logged_stage_) {
    RCLCPP_INFO(get_logger(), "Flight stage: %s", task_->stage_name().c_str());
    last_logged_stage_ = task_->stage_name();
  }
  if (task_->stage_name() == "SAFETY_LAND") {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Safety landing active (%s); waiting for fresh landed and disarmed confirmation",
      task_->result_reason().c_str());
  }

  if (result.publish_target) {
    auto target = result.target;
    target.header.stamp = now();
    target_pub_->publish(target);
  }

  if (result.send_command) {
    auto request = std::make_shared<uav_interfaces::srv::FlightCommand::Request>();
    request->command_id = result.command_id;
    request->command = result.command;
    const auto command_id = result.command_id;
    RCLCPP_INFO(
      get_logger(), "Dispatching command id=%u code=%u (dispatch is not state confirmation)",
      command_id, request->command);
    command_client_->async_send_request(
      request, [this, command_id](
        rclcpp::Client<uav_interfaces::srv::FlightCommand>::SharedFuture future)
      {
        try {
          const auto response = future.get();
          RCLCPP_INFO(
            get_logger(), "Command id=%u dispatched=%s: %s", command_id,
            response->dispatched ? "true" : "false", response->message.c_str());
          task_->command_dispatched(
            command_id, response->dispatched, response->message, Clock::now());
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Command id=%u service failed: %s", command_id, error.what());
          task_->command_dispatched(command_id, false, error.what(), Clock::now());
        }
      });
  }

  if (result.finished) {
    report_result(result);
    rclcpp::shutdown();
  }
}

void MissionNode::report_result(const FlightTask::Output & result)
{
  if (result_reported_) {
    return;
  }
  result_reported_ = true;
  if (task_->have_hover_measurements()) {
    const auto metrics =
      "hover_max_horizontal_error=" + std::to_string(task_->max_horizontal_error_m()) +
      " m, hover_max_height_error=" + std::to_string(task_->max_vertical_error_m()) +
      " m, hover_max_yaw_error=" + std::to_string(task_->max_yaw_error_rad()) +
      " rad, final_xy_displacement=" +
      std::to_string(task_->final_horizontal_displacement_m()) +
      " m, final_z_from_takeoff=" +
      std::to_string(task_->final_vertical_displacement_m()) + " m";
    if (result.passed) {
      RCLCPP_INFO(
        get_logger(), "PASS: Offboard, arm, 1 m takeoff, stable hold, 10 s hover, "
        "AUTO.LAND handover, landed and disarmed confirmed; %s", metrics.c_str());
    } else if (result.canceled) {
      RCLCPP_WARN(
        get_logger(), "CANCELED: safety landing completed; %s; %s",
        task_->result_reason().c_str(), metrics.c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(), "FAIL: %s; safety landing completed; %s",
        task_->result_reason().c_str(), metrics.c_str());
    }
  } else if (result.passed) {
    RCLCPP_INFO(get_logger(), "PASS: task completed; no hover position measurements were recorded");
  } else if (task_->have_position_measurement()) {
    const auto message =
      "last horizontal target error=" +
      std::to_string(task_->last_horizontal_target_error_m()) +
      " m, last height target error=" +
      std::to_string(task_->last_vertical_target_error_m()) +
      " m, final XY displacement=" +
      std::to_string(task_->final_horizontal_displacement_m()) +
      " m, final height from takeoff=" +
      std::to_string(task_->final_vertical_displacement_m()) + " m";
    if (result.canceled) {
      RCLCPP_WARN(
        get_logger(), "CANCELED: %s; %s", task_->result_reason().c_str(), message.c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(), "FAIL: %s; %s", task_->result_reason().c_str(), message.c_str());
    }
  } else if (result.canceled) {
    RCLCPP_WARN(get_logger(), "CANCELED: %s", task_->result_reason().c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "FAIL: %s; no hover position measurements were recorded",
      task_->result_reason().c_str());
  }
}

}  // namespace uav_mission

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<uav_mission::MissionNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("uav_mission"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
