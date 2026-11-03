#include "uav_mission/flight_task.hpp"

#include <algorithm>
#include <cmath>

namespace uav_mission
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double pose_yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool landed_on_ground(const uav_interfaces::msg::VehicleState & state)
{
  return state.landed_state_valid &&
    state.landed_state == uav_interfaces::msg::VehicleState::LANDED_ON_GROUND;
}

}  // namespace

FlightTask::FlightTask(const FlightTaskConfig & config)
: config_(config), stage_started_(Clock::now()), stage_name_("WAIT_READY")
{
}

void FlightTask::enter(Stage stage, Clock::time_point now_time)
{
  stage_ = stage;
  stage_started_ = now_time;
  pending_command_id_ = 0;
  pending_command_ = 0;
  command_request_sent_ = false;
  command_dispatched_ = false;
  command_dispatched_ok_ = false;
  command_dispatch_message_.clear();
  switch (stage) {
    case Stage::WAIT_READY: stage_name_ = "WAIT_READY"; break;
    case Stage::PREPARE_OFFBOARD: stage_name_ = "PREPARE_OFFBOARD"; break;
    case Stage::REQUEST_OFFBOARD: stage_name_ = "REQUEST_OFFBOARD"; break;
    case Stage::REQUEST_ARM: stage_name_ = "REQUEST_ARM"; break;
    case Stage::TAKEOFF:
      stage_name_ = "TAKEOFF";
      if (takeoff_started_ == Clock::time_point{}) {
        takeoff_started_ = now_time;
      }
      break;
    case Stage::STABILIZE: stage_name_ = "STABILIZE"; break;
    case Stage::HOVER:
      stage_name_ = "HOVER";
      hover_measurements_valid_ = true;
      max_horizontal_error_m_ = 0.0;
      max_vertical_error_m_ = 0.0;
      max_yaw_error_rad_ = 0.0;
      break;
    case Stage::REQUEST_LAND:
      stage_name_ = "REQUEST_LAND";
      landing_started_ = now_time;
      break;
    case Stage::REQUEST_DISARM: stage_name_ = "REQUEST_DISARM"; break;
    case Stage::LANDING: stage_name_ = "LANDING"; break;
    case Stage::SAFETY_LAND: stage_name_ = "SAFETY_LAND"; break;
    case Stage::FINISHED: stage_name_ = "FINISHED"; break;
  }
}

void FlightTask::begin_safety(Clock::time_point now_time)
{
  safety_retry_after_ = now_time;
  if (last_state_fresh_ && last_state_.position_valid) {
    const auto & p = last_state_.local_pose.pose.position;
    const double yaw = pose_yaw(last_state_.local_pose.pose.orientation);
    set_target(p.x, p.y, p.z, yaw);
  }
  enter(Stage::SAFETY_LAND, now_time);
}

void FlightTask::begin_failure(const std::string & reason, Clock::time_point now_time)
{
  if (failure_reason_.empty()) {
    failure_reason_ = reason;
  }
  if (flight_may_be_armed_) {
    begin_safety(now_time);
  } else {
    finish(false, false);
  }
}

void FlightTask::finish(bool passed, bool canceled)
{
  passed_ = passed;
  canceled_ = canceled;
  finished_ = true;
  enter(Stage::FINISHED, Clock::now());
}

void FlightTask::request_command(uint8_t command)
{
  pending_command_id_ = next_command_id_++;
  pending_command_ = command;
  command_request_sent_ = true;
  command_dispatched_ = false;
  command_dispatched_ok_ = false;
  command_started_ = Clock::now();
}

bool FlightTask::status_valid(
  const uav_interfaces::msg::VehicleState * state, bool state_stream_fresh) const
{
  return state && state_stream_fresh && state->state_valid && state->connected;
}

bool FlightTask::flight_state_valid(
  const uav_interfaces::msg::VehicleState * state, bool state_stream_fresh) const
{
  return status_valid(state, state_stream_fresh) && state->position_valid;
}

bool FlightTask::command_accepted(const uav_interfaces::msg::VehicleState * state) const
{
  return command_dispatched_ && command_dispatched_ok_ && state &&
    state->last_command_id == pending_command_id_ && state->command_completed &&
    state->command_accepted;
}

bool FlightTask::command_rejected(const uav_interfaces::msg::VehicleState * state) const
{
  return command_dispatched_ && command_dispatched_ok_ && state &&
    state->last_command_id == pending_command_id_ && state->command_completed &&
    !state->command_accepted;
}

double FlightTask::horizontal_error(const uav_interfaces::msg::VehicleState & state) const
{
  const auto & p = state.local_pose.pose.position;
  return std::hypot(p.x - home_x_, p.y - home_y_);
}

double FlightTask::vertical_error(const uav_interfaces::msg::VehicleState & state) const
{
  return std::abs(state.local_pose.pose.position.z - target_z_);
}

double FlightTask::yaw_error(const uav_interfaces::msg::VehicleState & state) const
{
  return std::abs(wrap_angle(pose_yaw(state.local_pose.pose.orientation) - home_yaw_));
}

void FlightTask::update_errors(const uav_interfaces::msg::VehicleState & state)
{
  max_horizontal_error_m_ = std::max(max_horizontal_error_m_, horizontal_error(state));
  max_vertical_error_m_ = std::max(max_vertical_error_m_, vertical_error(state));
  max_yaw_error_rad_ = std::max(max_yaw_error_rad_, yaw_error(state));
}

void FlightTask::set_target(double x, double y, double z, double yaw)
{
  target_.header.frame_id = "map";
  target_.pose.position.x = x;
  target_.pose.position.y = y;
  target_.pose.position.z = z;
  target_.pose.orientation.x = 0.0;
  target_.pose.orientation.y = 0.0;
  target_.pose.orientation.z = std::sin(yaw / 2.0);
  target_.pose.orientation.w = std::cos(yaw / 2.0);
  target_valid_ = true;
}

FlightTask::Output FlightTask::output(Clock::time_point)
{
  Output result;
  result.publish_target = target_valid_ && !finished_ && stage_ != Stage::SAFETY_LAND;
  result.target = target_;
  result.finished = finished_;
  result.passed = passed_;
  result.canceled = canceled_;
  return result;
}

FlightTask::Output FlightTask::update(
  const uav_interfaces::msg::VehicleState * state,
  bool state_stream_fresh, Clock::time_point now_time)
{
  if (state && state_stream_fresh) {
    last_state_ = *state;
    last_state_fresh_ = true;
    if (state->armed) {
      flight_may_be_armed_ = true;
    }
    if (state->position_valid) {
      final_horizontal_displacement_m_ = std::hypot(
        state->local_pose.pose.position.x - home_x_, state->local_pose.pose.position.y - home_y_);
      final_vertical_displacement_m_ =
        state->local_pose.pose.position.z - home_z_;
      if (target_valid_) {
        last_horizontal_target_error_m_ = horizontal_error(*state);
        last_vertical_target_error_m_ = vertical_error(*state);
        have_position_measurement_ = true;
      }
    }
  } else {
    last_state_fresh_ = false;
  }

  if (cancel_requested_ && stage_ != Stage::SAFETY_LAND && stage_ != Stage::FINISHED) {
    if (flight_may_be_armed_) {
      canceled_ = true;
      failure_reason_ = "任务收到取消请求";
      begin_safety(now_time);
    } else {
      finish(false, true);
    }
  }
  if (finished_) {
    return output(now_time);
  }

  if (stage_ == Stage::WAIT_READY) {
    if (status_valid(state, state_stream_fresh) && state->armed) {
      begin_failure("任务开始时飞控已经解锁", now_time);
      return output(now_time);
    }
    if (flight_state_valid(state, state_stream_fresh) && !state->control_inhibited && !state->armed &&
      landed_on_ground(*state))
    {
      const auto & pose = state->local_pose.pose;
      home_x_ = pose.position.x;
      home_y_ = pose.position.y;
      home_z_ = pose.position.z;
      home_yaw_ = pose_yaw(pose.orientation);
      target_z_ = home_z_ + config_.takeoff_height_m;
      set_target(home_x_, home_y_, home_z_, home_yaw_);
      enter(Stage::PREPARE_OFFBOARD, now_time);
    } else if (std::chrono::duration<double>(now_time - stage_started_).count() >=
      config_.ready_timeout_s)
    {
      begin_failure("等待飞控连接、有效新鲜定位和地面状态超时", now_time);
    }
    return output(now_time);
  }

  if (stage_ == Stage::SAFETY_LAND) {
    Output result = output(now_time);
    if (!status_valid(state, state_stream_fresh)) {
      result.publish_target = false;
      return result;
    }

    const bool on_ground = landed_on_ground(*state);
    if (on_ground && !state->armed) {
      finish(false, canceled_);
      return output(now_time);
    }

    if (on_ground && state->armed) {
      result.publish_target = false;
      if (pending_command_id_ != 0 && (command_rejected(state) ||
        std::chrono::duration<double>(now_time - command_started_).count() >=
        config_.command_timeout_s))
      {
        pending_command_id_ = 0;
        command_request_sent_ = false;
        safety_retry_after_ = now_time + std::chrono::seconds(1);
      }
      if (pending_command_id_ == 0 && now_time >= safety_retry_after_) {
        request_command(uav_interfaces::srv::FlightCommand::Request::DISARM);
        result.send_command = true;
        result.command_id = pending_command_id_;
        result.command = pending_command_;
      }
      return result;
    }

    const bool auto_land = state->mode == "AUTO.LAND";
    result.publish_target = !auto_land && state->position_valid && state->mode == "OFFBOARD";
    if (state->position_valid) {
      const auto & p = state->local_pose.pose.position;
      set_target(p.x, p.y, p.z, pose_yaw(state->local_pose.pose.orientation));
      result.target = target_;
    }
    if (auto_land) {
      return result;
    }

    if (pending_command_id_ != 0 &&
      (command_rejected(state) ||
      std::chrono::duration<double>(now_time - command_started_).count() >=
      config_.command_timeout_s))
    {
      pending_command_id_ = 0;
      command_request_sent_ = false;
      safety_retry_after_ = now_time + std::chrono::seconds(1);
    }
    if (pending_command_id_ == 0 && state->position_valid && now_time >= safety_retry_after_) {
      request_command(uav_interfaces::srv::FlightCommand::Request::LAND);
      result.send_command = true;
      result.command_id = pending_command_id_;
      result.command = pending_command_;
    }
    return result;
  }

  if (!status_valid(state, state_stream_fresh)) {
    begin_failure("飞控连接状态未知或过期", now_time);
    return output(now_time);
  }

  // PX4 controls descent after AUTO.LAND; the remaining stages only need
  // vehicle and landed state, not a live offboard position estimate.
  if (!state->position_valid && stage_ != Stage::LANDING && stage_ != Stage::REQUEST_DISARM)
  {
    begin_failure("本地位置无效或过期", now_time);
    return output(now_time);
  }

  const bool land_stage = stage_ == Stage::REQUEST_LAND || stage_ == Stage::LANDING ||
    stage_ == Stage::REQUEST_DISARM;
  if (state->control_inhibited && !land_stage) {
    begin_failure("控制后端禁止继续任务: " + state->control_message, now_time);
    return output(now_time);
  }
  if (land_stage) {
    if (stage_ == Stage::REQUEST_LAND && state->mode != "OFFBOARD" &&
      state->mode != "AUTO.LAND")
    {
      begin_failure("降落请求前 Offboard 模式意外改变为 " + state->mode, now_time);
      return output(now_time);
    }
    if (stage_ != Stage::REQUEST_LAND && state->mode != "AUTO.LAND" &&
      state->landed_state_valid && !landed_on_ground(*state))
    {
      begin_failure("降落期间飞控模式离开 AUTO.LAND", now_time);
      return output(now_time);
    }
  } else if (stage_ != Stage::PREPARE_OFFBOARD && stage_ != Stage::REQUEST_OFFBOARD &&
    state->mode != "OFFBOARD") {
    begin_failure("Offboard 模式意外改变为 " + state->mode, now_time);
    return output(now_time);
  }

  if ((stage_ == Stage::TAKEOFF || stage_ == Stage::STABILIZE || stage_ == Stage::HOVER) &&
    !state->armed)
  {
    begin_failure("飞行阶段飞控报告未解锁", now_time);
    return output(now_time);
  }

  Output result = output(now_time);
  const auto elapsed = std::chrono::duration<double>(now_time - stage_started_).count();
  switch (stage_) {
    case Stage::PREPARE_OFFBOARD:
      if (elapsed >= 1.2) {
        enter(Stage::REQUEST_OFFBOARD, now_time);
      }
      break;

    case Stage::REQUEST_OFFBOARD:
      if (pending_command_id_ == 0) {
        request_command(uav_interfaces::srv::FlightCommand::Request::SET_OFFBOARD);
        result.send_command = true;
        result.command_id = pending_command_id_;
        result.command = pending_command_;
      } else if (command_rejected(state)) {
        begin_failure("MAVROS/PX4 拒绝 Offboard 请求: " + state->command_message, now_time);
      } else if (command_accepted(state) && state->mode == "OFFBOARD") {
        enter(Stage::REQUEST_ARM, now_time);
      } else if (elapsed >= config_.command_timeout_s) {
        begin_failure("Offboard 命令接受或模式确认超时", now_time);
      }
      break;

    case Stage::REQUEST_ARM:
      if (pending_command_id_ == 0) {
        flight_may_be_armed_ = true;
        request_command(uav_interfaces::srv::FlightCommand::Request::ARM);
        result.send_command = true;
        result.command_id = pending_command_id_;
        result.command = pending_command_;
      } else if (command_rejected(state)) {
        begin_failure("MAVROS/PX4 拒绝解锁请求: " + state->command_message, now_time);
      } else if (command_accepted(state) && state->armed) {
        set_target(home_x_, home_y_, target_z_, home_yaw_);
        result.target = target_;
        enter(Stage::TAKEOFF, now_time);
      } else if (elapsed >= config_.command_timeout_s) {
        begin_failure("解锁命令接受或解锁状态确认超时", now_time);
      }
      break;

    case Stage::TAKEOFF:
      if (std::chrono::duration<double>(now_time - takeoff_started_).count() >=
        config_.takeoff_timeout_s)
      {
        begin_failure("起飞到 1.0 米目标超时", now_time);
      } else if (horizontal_error(*state) <= config_.horizontal_tolerance_m &&
        vertical_error(*state) <= config_.vertical_tolerance_m)
      {
        stable_since_ = now_time;
        have_stable_since_ = true;
        enter(Stage::STABILIZE, now_time);
      }
      break;

    case Stage::STABILIZE:
      update_errors(*state);
      if (horizontal_error(*state) > config_.horizontal_tolerance_m ||
        vertical_error(*state) > config_.vertical_tolerance_m)
      {
        have_stable_since_ = false;
        enter(Stage::TAKEOFF, now_time);
      } else if (!have_stable_since_) {
        stable_since_ = now_time;
        have_stable_since_ = true;
      } else if (std::chrono::duration<double>(now_time - stable_since_).count() >=
        config_.stable_duration_s)
      {
        enter(Stage::HOVER, now_time);
      }
      if (std::chrono::duration<double>(now_time - takeoff_started_).count() >=
        config_.takeoff_timeout_s)
      {
        begin_failure("起飞后连续稳定 2 秒超时", now_time);
      }
      break;

    case Stage::HOVER:
      if (config_.relay_mode) {
        // Relay targets are selected by MissionNode after takeoff stabilizes.
        break;
      }
      update_errors(*state);
      if (horizontal_error(*state) > config_.horizontal_tolerance_m ||
        vertical_error(*state) > config_.vertical_tolerance_m)
      {
        begin_failure("10 秒悬停期间位置误差超限", now_time);
      } else if (elapsed >= config_.hover_duration_s) {
        enter(Stage::REQUEST_LAND, now_time);
      }
      break;

    case Stage::REQUEST_LAND:
      if (pending_command_id_ == 0) {
        request_command(uav_interfaces::srv::FlightCommand::Request::LAND);
        result.send_command = true;
        result.command_id = pending_command_id_;
        result.command = pending_command_;
      } else if (command_rejected(state)) {
        begin_failure("MAVROS/PX4 拒绝降落请求: " + state->command_message, now_time);
      } else if (command_accepted(state) && state->mode == "AUTO.LAND") {
        enter(Stage::LANDING, now_time);
      } else if (std::chrono::duration<double>(now_time - command_started_).count() >=
        config_.command_timeout_s)
      {
        begin_failure("降落命令服务接受或 AUTO.LAND 模式确认超时", now_time);
      }
      break;

    case Stage::REQUEST_DISARM:
      if (!state->landed_state_valid) {
        result.publish_target = false;
        if (std::chrono::duration<double>(now_time - landing_started_).count() >=
          config_.landing_timeout_s)
        {
          begin_failure("等待飞控着陆状态超时", now_time);
        }
      } else if (!landed_on_ground(*state)) {
        begin_failure("解锁状态确认期间地面着陆状态丢失", now_time);
      } else if (!state->armed) {
        finish(true, false);
      } else if (pending_command_id_ == 0) {
        request_command(uav_interfaces::srv::FlightCommand::Request::DISARM);
        result.send_command = true;
        result.command_id = pending_command_id_;
        result.command = pending_command_;
      } else if (command_rejected(state)) {
        begin_failure("MAVROS/PX4 拒绝上锁请求: " + state->command_message, now_time);
      } else if (command_accepted(state) && !state->armed) {
        finish(true, false);
      } else if (elapsed >= config_.command_timeout_s) {
        begin_failure("上锁状态确认超时", now_time);
      }
      break;

    case Stage::LANDING:
      if (landed_on_ground(*state)) {
        result.publish_target = false;
        if (!state->armed) {
          finish(true, false);
        } else {
          enter(Stage::REQUEST_DISARM, now_time);
        }
      } else if (std::chrono::duration<double>(now_time - landing_started_).count() >=
        config_.landing_timeout_s)
      {
        begin_failure("等待飞控报告着陆超时", now_time);
      }
      result.publish_target = state->mode != "AUTO.LAND";
      break;

    case Stage::WAIT_READY:
    case Stage::SAFETY_LAND:
    case Stage::FINISHED:
      break;
  }

  result.finished = finished_;
  result.passed = passed_;
  result.canceled = canceled_;
  if (stage_ == Stage::SAFETY_LAND) {
    result = output(now_time);
    result.publish_target = status_valid(state, state_stream_fresh) && state->position_valid &&
      state->mode == "OFFBOARD";
  }
  if (finished_) {
    result.publish_target = false;
  }
  return result;
}

void FlightTask::command_dispatched(
  uint32_t command_id, bool dispatched, const std::string & message, Clock::time_point now_time)
{
  if (command_id != pending_command_id_) {
    return;
  }
  command_dispatched_ = true;
  command_dispatched_ok_ = dispatched;
  command_dispatch_message_ = message;
  if (!dispatched) {
    if (stage_ == Stage::SAFETY_LAND) {
      pending_command_id_ = 0;
      command_request_sent_ = false;
      safety_retry_after_ = now_time + std::chrono::seconds(1);
    } else {
      begin_failure("控制命令未能转发: " + message, now_time);
    }
  }
}

void FlightTask::cancel(Clock::time_point now_time)
{
  if (!finished_) {
    cancel_requested_ = true;
    if (flight_may_be_armed_) {
      canceled_ = true;
      failure_reason_ = "任务收到取消请求";
      begin_safety(now_time);
    } else {
      finish(false, true);
    }
  }
}

const std::string & FlightTask::stage_name() const
{
  return stage_name_;
}

const std::string & FlightTask::result_reason() const
{
  return failure_reason_;
}

double FlightTask::max_horizontal_error_m() const { return max_horizontal_error_m_; }
double FlightTask::max_vertical_error_m() const { return max_vertical_error_m_; }
double FlightTask::max_yaw_error_rad() const { return max_yaw_error_rad_; }
double FlightTask::final_horizontal_displacement_m() const
{
  return final_horizontal_displacement_m_;
}
double FlightTask::final_vertical_displacement_m() const { return final_vertical_displacement_m_; }
bool FlightTask::have_hover_measurements() const { return hover_measurements_valid_; }
double FlightTask::last_horizontal_target_error_m() const
{
  return last_horizontal_target_error_m_;
}
double FlightTask::last_vertical_target_error_m() const { return last_vertical_target_error_m_; }
bool FlightTask::have_position_measurement() const { return have_position_measurement_; }

}  // namespace uav_mission
