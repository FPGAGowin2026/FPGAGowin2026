#include "uav_control/mavros_interface.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>

namespace uav_control
{

MavrosInterface::MavrosInterface(const rclcpp::NodeOptions & options)
: Node("uav_control", options)
{
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
  state_timeout_s_ = declare_parameter<double>("state_timeout_s", 1.0);
  position_timeout_s_ = declare_parameter<double>("position_timeout_s", 0.5);
  target_timeout_s_ = declare_parameter<double>("target_timeout_s", 1.0);
  land_handover_timeout_s_ = declare_parameter<double>("land_handover_timeout_s", 3.0);
  command_queue_timeout_s_ = declare_parameter<double>("command_queue_timeout_s", 3.0);
  if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 2.0 || publish_rate_hz_ > 100.0 ||
    !std::isfinite(state_timeout_s_) || state_timeout_s_ <= 0.0 ||
    !std::isfinite(position_timeout_s_) || position_timeout_s_ <= 0.0 ||
    !std::isfinite(target_timeout_s_) || target_timeout_s_ <= 0.0 ||
    !std::isfinite(land_handover_timeout_s_) || land_handover_timeout_s_ <= 0.0 ||
    !std::isfinite(command_queue_timeout_s_) || command_queue_timeout_s_ <= 0.0)
  {
    throw std::invalid_argument("Control rates and freshness/timeout parameters must be positive");
  }

  state_sub_ = create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", 10,
    std::bind(&MavrosInterface::state_callback, this, std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
    std::bind(&MavrosInterface::pose_callback, this, std::placeholders::_1));
  extended_state_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
    "/mavros/extended_state", 10,
    std::bind(&MavrosInterface::extended_state_callback, this, std::placeholders::_1));
  target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/uav_control/target", 10,
    std::bind(&MavrosInterface::target_callback, this, std::placeholders::_1));

  setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/setpoint_position/local", 10);
  state_pub_ = create_publisher<uav_interfaces::msg::VehicleState>(
    "/uav_control/state", 10);

  set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  arming_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  land_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
  command_service_ = create_service<FlightCommand>(
    "/uav_control/command",
    std::bind(
      &MavrosInterface::command_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&MavrosInterface::control_tick, this));

  RCLCPP_INFO(
    get_logger(),
    "MAVROS control ready: %.1f Hz setpoint loop; target topic /uav_control/target",
    publish_rate_hz_);
}

void MavrosInterface::state_callback(const mavros_msgs::msg::State::SharedPtr msg)
{
  if (!have_state_ || msg->connected != fcu_state_.connected ||
    msg->mode != fcu_state_.mode || msg->armed != fcu_state_.armed)
  {
    RCLCPP_INFO(
      get_logger(), "FCU %s, mode=%s, armed=%s",
      msg->connected ? "CONNECTED" : "DISCONNECTED", msg->mode.c_str(),
      msg->armed ? "true" : "false");
  }
  fcu_state_ = *msg;
  state_rx_ = Clock::now();
  have_state_ = true;
}

void MavrosInterface::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  local_pose_ = *msg;
  pose_rx_ = Clock::now();
  const auto & p = msg->pose.position;
  const auto & q = msg->pose.orientation;
  const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  pose_valid_ = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
    std::isfinite(q_norm) && q_norm > 0.5;
}

void MavrosInterface::extended_state_callback(
  const mavros_msgs::msg::ExtendedState::SharedPtr msg)
{
  extended_state_ = *msg;
  extended_state_rx_ = Clock::now();
  have_extended_state_ = true;
}

void MavrosInterface::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const auto & p = msg->pose.position;
  const auto & q = msg->pose.orientation;
  const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (msg->header.frame_id != "map" || !std::isfinite(p.x) || !std::isfinite(p.y) ||
    !std::isfinite(p.z) || !std::isfinite(q_norm) || q_norm < 1e-6)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring invalid target; require finite map-frame pose and nonzero orientation");
    return;
  }

  target_ = *msg;
  target_.header.stamp = now();
  target_rx_ = Clock::now();
  have_target_ = true;
}

void MavrosInterface::command_callback(
  const std::shared_ptr<FlightCommand::Request> request,
  std::shared_ptr<FlightCommand::Response> response)
{
  if (request->command_id == 0 || command_pending_ || safety_latched_) {
    response->dispatched = false;
    response->message = safety_latched_ ? "control safety latch is active" :
      (command_pending_ ? "another MAVROS command is pending" : "command_id 0 is reserved");
    return;
  }

  response->dispatched = send_command(request->command_id, request->command, false);
  response->message = response->dispatched ?
    (queued_command_ ? "queued while waiting for MAVROS service" : "forwarded to MAVROS service") :
    "command unsupported or another command pending";
}

bool MavrosInterface::command_service_ready(uint8_t command) const
{
  if (command == FlightCommand::Request::SET_OFFBOARD) {
    return set_mode_client_->service_is_ready();
  }
  if (command == FlightCommand::Request::ARM || command == FlightCommand::Request::DISARM) {
    return arming_client_->service_is_ready();
  }
  if (command == FlightCommand::Request::LAND) {
    return land_client_->service_is_ready();
  }
  return false;
}

bool MavrosInterface::send_command(uint32_t command_id, uint8_t command, bool internal)
{
  if (command_pending_) {
    return false;
  }

  if (command_service_ready(command)) {
    // Continue below and send the request now.
  } else if (command == FlightCommand::Request::SET_OFFBOARD ||
    command == FlightCommand::Request::ARM || command == FlightCommand::Request::DISARM ||
    command == FlightCommand::Request::LAND)
  {
    queued_command_ = true;
    queued_command_id_ = command_id;
    queued_command_code_ = command;
    queued_command_internal_ = internal;
    queued_command_started_ = Clock::now();
    command_pending_ = true;
    RCLCPP_WARN(
      get_logger(), "Queued MAVROS command id=%u code=%u until its service is ready",
      command_id, command);
    return true;
  } else {
    return false;
  }

  if (command == FlightCommand::Request::SET_OFFBOARD) {
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = "OFFBOARD";
    command_pending_ = true;
    set_mode_client_->async_send_request(
      request, [this, command_id, internal](
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
      {
        try {
          const auto result = future.get();
          finish_command(command_id, result->mode_sent,
            result->mode_sent ? "MAVROS accepted OFFBOARD request" :
            "MAVROS rejected OFFBOARD request");
        } catch (const std::exception & error) {
          finish_command(command_id, false, error.what());
        }
        if (internal) {
          RCLCPP_INFO(get_logger(), "Internal command result id=%u: %s", command_id,
            command_message_.c_str());
        }
      });
    return true;
  }

  if (command == FlightCommand::Request::ARM || command == FlightCommand::Request::DISARM) {
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = command == FlightCommand::Request::ARM;
    command_pending_ = true;
    arming_client_->async_send_request(
      request, [this, command_id, internal, arm = request->value](
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
      {
        try {
          const auto result = future.get();
          finish_command(command_id, result->success,
            result->success ? (arm ? "MAVROS accepted arm request" :
            "MAVROS accepted disarm request") : "MAVROS rejected arming request");
        } catch (const std::exception & error) {
          finish_command(command_id, false, error.what());
        }
        if (internal) {
          RCLCPP_INFO(get_logger(), "Internal command result id=%u: %s", command_id,
            command_message_.c_str());
        }
      });
    return true;
  }

  if (command == FlightCommand::Request::LAND) {
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->min_pitch = 0.0F;
    request->yaw = 0.0F;
    request->latitude = 0.0;
    request->longitude = 0.0;
    request->altitude = 0.0;
    command_pending_ = true;
    land_client_->async_send_request(
      request, [this, command_id, internal](
        rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future)
      {
        try {
          const auto result = future.get();
          finish_command(command_id, result->success,
            result->success ? "MAVROS accepted land request" : "MAVROS rejected land request");
        } catch (const std::exception & error) {
          finish_command(command_id, false, error.what());
        }
        if (internal) {
          RCLCPP_INFO(get_logger(), "Internal land result: %s", command_message_.c_str());
        }
      });
    return true;
  }
  return false;
}

void MavrosInterface::finish_command(
  uint32_t command_id, bool accepted, const std::string & message)
{
  command_pending_ = false;
  last_command_id_ = command_id;
  command_completed_ = true;
  command_accepted_ = accepted;
  command_message_ = message;
  RCLCPP_INFO(
    get_logger(), "MAVROS command id=%u completed: accepted=%s (%s)", command_id,
    accepted ? "true" : "false", message.c_str());
}

bool MavrosInterface::state_fresh(const Clock::time_point now_time) const
{
  return have_state_ &&
    std::chrono::duration<double>(now_time - state_rx_).count() <= state_timeout_s_;
}

bool MavrosInterface::position_fresh(const Clock::time_point now_time) const
{
  return pose_valid_ &&
    std::chrono::duration<double>(now_time - pose_rx_).count() <= position_timeout_s_;
}

bool MavrosInterface::landed_state_fresh(const Clock::time_point now_time) const
{
  return have_extended_state_ &&
    std::chrono::duration<double>(now_time - extended_state_rx_).count() <= state_timeout_s_;
}

bool MavrosInterface::target_fresh(const Clock::time_point now_time) const
{
  return have_target_ &&
    std::chrono::duration<double>(now_time - target_rx_).count() <= target_timeout_s_;
}

bool MavrosInterface::airborne_or_unknown(const Clock::time_point now_time) const
{
  return !landed_state_fresh(now_time) ||
    extended_state_.landed_state != uav_interfaces::msg::VehicleState::LANDED_ON_GROUND;
}

void MavrosInterface::publish_state(const Clock::time_point now_time)
{
  uav_interfaces::msg::VehicleState state;
  state.state_valid = state_fresh(now_time);
  state.connected = fcu_state_.connected;
  state.armed = fcu_state_.armed;
  state.mode = fcu_state_.mode;
  state.position_valid = position_fresh(now_time);
  state.local_pose = local_pose_;
  state.landed_state_valid = landed_state_fresh(now_time);
  state.landed_state = have_extended_state_ ? extended_state_.landed_state : 0;
  state.last_command_id = last_command_id_;
  state.command_completed = command_completed_;
  state.command_accepted = command_accepted_;
  state.command_message = command_message_;
  state_pub_->publish(state);
}

void MavrosInterface::control_tick()
{
  const auto now_time = Clock::now();
  publish_state(now_time);

  if (queued_command_) {
    const double queued_age = std::chrono::duration<double>(
      now_time - queued_command_started_).count();
    if (command_service_ready(queued_command_code_)) {
      const auto command_id = queued_command_id_;
      const auto command_code = queued_command_code_;
      const bool internal = queued_command_internal_;
      queued_command_ = false;
      command_pending_ = false;
      RCLCPP_INFO(get_logger(), "MAVROS service ready; forwarding queued command id=%u", command_id);
      if (!send_command(command_id, command_code, internal)) {
        finish_command(command_id, false, "MAVROS command could not be forwarded");
      }
    } else if (queued_age >= command_queue_timeout_s_) {
      const auto command_id = queued_command_id_;
      const bool internal = queued_command_internal_;
      queued_command_ = false;
      finish_command(command_id, false, "MAVROS service did not become ready before timeout");
      if (internal) {
        RCLCPP_ERROR(get_logger(), "Internal queued MAVROS command expired");
      }
    }
  }

  const bool connected_fresh = state_fresh(now_time) && fcu_state_.connected;
  const bool airborne = fcu_state_.armed && airborne_or_unknown(now_time);
  const bool target_current = target_fresh(now_time);
  const bool local_position_current = position_fresh(now_time);
  const bool target_expected = fcu_state_.mode != "AUTO.LAND";
  if (!safety_latched_ && airborne &&
    (!connected_fresh || !local_position_current || (target_expected && !target_current)))
  {
    safety_latched_ = true;
    safety_started_ = now_time;
    last_internal_land_request_ = Clock::time_point{};
    RCLCPP_ERROR(
      get_logger(),
      "Safety latch: %s; target age limit %.2f s, position age limit %.2f s",
      !connected_fresh ? "FCU state is disconnected or stale" :
      (!local_position_current ? "local position is invalid or stale" : "mission target timed out"),
      target_timeout_s_, position_timeout_s_);
  }

  if (!safety_latched_) {
    if (target_current) {
      setpoint_pub_->publish(target_);
    }
    return;
  }

  // Keep the last setpoint only while the FCU is still in Offboard and a fresh
  // local pose is available. If the land handover fails, stop the stream so
  // PX4's configured Offboard-loss failsafe can take over.
  const double safety_age = std::chrono::duration<double>(now_time - safety_started_).count();
  if (connected_fresh && local_position_current && fcu_state_.mode == "OFFBOARD" &&
    safety_age <= land_handover_timeout_s_ && have_target_)
  {
    setpoint_pub_->publish(target_);
  }

  if (connected_fresh && local_position_current && airborne &&
    fcu_state_.mode != "AUTO.LAND" &&
    (last_internal_land_request_ == Clock::time_point{} ||
    std::chrono::duration<double>(now_time - last_internal_land_request_).count() >=
    land_handover_timeout_s_))
  {
    RCLCPP_ERROR(get_logger(), "Target/control lost: requesting MAVROS land while localization is fresh");
    if (send_command(0, FlightCommand::Request::LAND, true)) {
      last_internal_land_request_ = now_time;
    } else {
      last_internal_land_request_ = now_time;
      RCLCPP_ERROR(get_logger(), "Land service unavailable; PX4 Offboard-loss failsafe remains enabled");
    }
  }
}

}  // namespace uav_control
