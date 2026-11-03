#include "uav_mission/relay_navigation.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace uav_mission
{
namespace
{

constexpr std::int64_t kFutureToleranceNs = 50000000LL;

std::optional<std::int64_t> stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) return std::nullopt;
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
    std::isfinite(norm2) && norm2 > 1e-12;
}

std::optional<std::pair<std::int64_t, std::int64_t>> spline_lifetime(
  const traj_utils::msg::Bspline & spline)
{
  const auto start = stamp_ns(spline.start_time);
  if (!start || *start <= 0 || spline.order != 3 || spline.pos_pts.size() < 4 ||
      spline.knots.size() != spline.pos_pts.size() + static_cast<std::size_t>(spline.order) + 1)
    return std::nullopt;
  for (std::size_t i = 0; i < spline.knots.size(); ++i) {
    if (!std::isfinite(spline.knots[i]) || (i && spline.knots[i] < spline.knots[i - 1]))
      return std::nullopt;
  }
  for (const auto & point : spline.pos_pts) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      return std::nullopt;
  }
  const auto order = static_cast<std::size_t>(spline.order);
  const double seconds = spline.knots[spline.knots.size() - order - 1] - spline.knots[order];
  if (!std::isfinite(seconds) || seconds <= 0.0 ||
      seconds > static_cast<double>(std::numeric_limits<std::int64_t>::max() - *start) / 1e9)
    return std::nullopt;
  const auto duration = static_cast<std::int64_t>(std::llround(seconds * 1e9));
  if (duration <= 0) return std::nullopt;
  return std::make_pair(*start, *start + duration);
}

}  // namespace

RelayNavigation::RelayNavigation(rclcpp::Node & node)
: node_(node), tf_buffer_(node.get_clock()), tf_listener_(tf_buffer_, &node, false)
{
  robot_id_ = node_.declare_parameter<std::string>("relay_robot_id", "uav_01");
  map_frame_ = node_.declare_parameter<std::string>("relay_frame", "map");
  arrival_tolerance_m_ = node_.declare_parameter<double>("relay_arrival_tolerance_m", 0.25);
  command_timeout_s_ = node_.declare_parameter<double>("relay_command_timeout_s", 0.3);
  sensor_timeout_s_ = node_.declare_parameter<double>("relay_sensor_timeout_s", 0.5);
  if (robot_id_.empty() || map_frame_ != "map" || !std::isfinite(arrival_tolerance_m_) ||
      arrival_tolerance_m_ <= 0 || !std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0 ||
      !std::isfinite(sensor_timeout_s_) || sensor_timeout_s_ <= 0) {
    throw std::invalid_argument("Invalid relay parameters; uav_control currently requires frame map");
  }

  goal_sub_ = node_.create_subscription<Goal>("/swarm/uav/relay_goal", 10,
    std::bind(&RelayNavigation::on_goal, this, std::placeholders::_1));
  association_sub_ = node_.create_subscription<std_msgs::msg::UInt64MultiArray>(
    "/uav_ego/trajectory_goal", 10,
    std::bind(&RelayNavigation::on_association, this, std::placeholders::_1));
  spline_sub_ = node_.create_subscription<Bspline>("/uav_ego/planning/bspline", 10,
    std::bind(&RelayNavigation::on_spline, this, std::placeholders::_1));
  command_sub_ = node_.create_subscription<Command>("/uav_ego/position_cmd", 50,
    std::bind(&RelayNavigation::on_command, this, std::placeholders::_1));
  sensor_sub_ = node_.create_subscription<std_msgs::msg::Bool>("/uav_ego/sensor_ready", 1,
    std::bind(&RelayNavigation::on_sensor_ready, this, std::placeholders::_1));
  planner_goal_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>("/uav_ego/goal", 10);
  status_pub_ = node_.create_publisher<Status>("/swarm/uav/relay_status", 10);
}

void RelayNavigation::on_goal(const Goal::SharedPtr message)
{
  if (message->robot_id != robot_id_) return;
  const auto stamp = stamp_ns(message->header.stamp);
  const auto now_ns = node_.now().nanoseconds();
  const auto & ttl = message->valid_for;
  const auto duration = static_cast<std::int64_t>(ttl.sec) * 1000000000LL + ttl.nanosec;
  if (!message->task_id || message->header.frame_id.empty() || !stamp || *stamp <= 0 ||
      ttl.sec < 0 || ttl.nanosec >= 1000000000u || duration <= 0 ||
      *stamp - now_ns > kFutureToleranceNs || duration > std::numeric_limits<std::int64_t>::max() - *stamp ||
      now_ns - *stamp >= duration || !finite_pose(message->target)) {
    publish_status(message->task_id, Status::FAILED, "目标无效或已过期");
    return;
  }
  if (have_goal_order_ && (*stamp < last_goal_source_ns_ ||
      (*stamp == last_goal_source_ns_ && message->task_id <= last_goal_id_))) {
    // Same task and stamp is an idempotent repeat; older samples cannot take control.
    if (active_ && active_->original.task_id == message->task_id && active_->source_ns == *stamp)
      publish_status(message->task_id, active_->state, active_->reason);
    return;
  }

  if (active_ && !active_->terminal) finish(Status::CANCELED, "已由新目标替换");
  if (current_pose_) hold_ = current_pose_;
  ActiveGoal goal;
  goal.original = *message;
  goal.source_ns = *stamp;
  goal.valid_until_ns = *stamp + duration;
  goal.reason = "等待坐标变换和起飞稳定";
  active_ = std::move(goal);
  have_goal_order_ = true;
  last_goal_source_ns_ = *stamp;
  last_goal_id_ = message->task_id;
  trajectories_.clear();
  was_executing_ = false;
  publish_status(message->task_id, Status::ACCEPTED, active_->reason);
}

void RelayNavigation::on_association(const std_msgs::msg::UInt64MultiArray::SharedPtr message)
{
  if (!active_ || active_->terminal || !active_->dispatched || message->data.size() != 2 ||
      message->data[0] != active_->token_ns || message->data[1] == 0 ||
      message->data[1] > std::numeric_limits<std::uint32_t>::max()) return;
  trajectories_[static_cast<std::uint32_t>(message->data[1])].goal_token = message->data[0];
  active_->has_association = true;
  trim_trajectories();
}

void RelayNavigation::on_spline(const Bspline::SharedPtr message)
{
  if (message->traj_id <= 0 ||
      static_cast<std::uint64_t>(message->traj_id) > std::numeric_limits<std::uint32_t>::max()) return;
  const auto lifetime = spline_lifetime(*message);
  if (!lifetime) return;
  const auto id = static_cast<std::uint32_t>(message->traj_id);
  if (last_spline_id_) {
    if (lifetime->first < last_spline_start_ns_) return;
    if (lifetime->first > last_spline_start_ns_ && id <= *last_spline_id_) {
      if (active_ && active_->dispatched && !active_->terminal)
        finish(Status::FAILED, "规划器轨迹编号回退，请重新发送目标");
      trajectories_.clear();
    }
  }
  last_spline_id_ = id;
  last_spline_start_ns_ = lifetime->first;
  if (!active_ || active_->terminal || !active_->dispatched) return;
  trajectories_[id].lifetime = *lifetime;
  trim_trajectories();
}

void RelayNavigation::on_command(const Command::SharedPtr message)
{
  const auto & p = message->position;
  if (!active_ || active_->terminal || !active_->dispatched || message->trajectory_id == 0 ||
      message->header.frame_id.empty() || !std::isfinite(message->yaw) ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
  const auto stamp = stamp_ns(message->header.stamp);
  if (!stamp || *stamp <= 0) return;
  auto & entry = trajectories_[message->trajectory_id];
  if (entry.command && *stamp <= entry.command_stamp_ns) return;
  entry.command = *message;
  entry.command_stamp_ns = *stamp;
  entry.command_received = Clock::now();
  trim_trajectories();
}

void RelayNavigation::on_sensor_ready(const std_msgs::msg::Bool::SharedPtr message)
{
  sensor_ready_ = message->data;
  sensor_received_ = Clock::now();
}

bool RelayNavigation::transform_to_map(const geometry_msgs::msg::PoseStamped & input,
                                       geometry_msgs::msg::PoseStamped & output) const
{
  if (input.header.frame_id.empty()) return false;
  try {
    if (input.header.frame_id == map_frame_) {
      output = input;
    } else {
      // The listener is serviced by this node's executor; query its buffer without blocking it.
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_, input.header.frame_id, tf2_ros::fromRclcpp(rclcpp::Time(input.header.stamp)));
      tf2::doTransform(input, output, transform);
      if (!finite_pose(output.pose)) return false;
    }
    auto & q = output.pose.orientation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    q.x /= norm; q.y /= norm; q.z /= norm; q.w /= norm;
    return output.header.frame_id == map_frame_;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

bool RelayNavigation::current_pose(const uav_interfaces::msg::VehicleState * state, bool state_fresh,
                                   geometry_msgs::msg::PoseStamped & output) const
{
  if (!state || !state_fresh || !state->position_valid) return false;
  return transform_to_map(state->local_pose, output);
}

bool RelayNavigation::sensors_fresh(Clock::time_point now) const
{
  // The bridge already checks the odometry, depth, timestamps and camera TF.
  // Here only check that its readiness stream itself is still alive.
  return sensor_ready_ && sensor_received_ != Clock::time_point{} &&
    std::chrono::duration<double>(now - sensor_received_).count() <= sensor_timeout_s_;
}

bool RelayNavigation::trajectory_target(std::int64_t now_ns, Clock::time_point now,
                                        geometry_msgs::msg::PoseStamped & output) const
{
  if (!active_ || active_->terminal || !active_->dispatched)
    return false;
  bool found = false;
  std::int64_t selected_start = -1;
  for (const auto & item : trajectories_) {
    const auto & entry = item.second;
    if (!entry.goal_token || *entry.goal_token != active_->token_ns || !entry.lifetime || !entry.command)
      continue;
    const auto & lifetime = *entry.lifetime;
    const auto stamp = entry.command_stamp_ns;
    if (lifetime.first < static_cast<std::int64_t>(active_->token_ns) ||
        now_ns < lifetime.first || now_ns >= lifetime.second || stamp < lifetime.first ||
        stamp >= lifetime.second || stamp > now_ns ||
        static_cast<double>(now_ns - stamp) > command_timeout_s_ * 1e9 ||
        entry.command_received == Clock::time_point{} ||
        std::chrono::duration<double>(now - entry.command_received).count() > command_timeout_s_ ||
        entry.command->trajectory_flag != Command::TRAJECTORY_STATUS_READY) continue;
    geometry_msgs::msg::PoseStamped candidate;
    candidate.header = entry.command->header;
    candidate.pose.position = entry.command->position;
    candidate.pose.orientation.z = std::sin(entry.command->yaw * 0.5);
    candidate.pose.orientation.w = std::cos(entry.command->yaw * 0.5);
    geometry_msgs::msg::PoseStamped transformed;
    if (!transform_to_map(candidate, transformed)) continue;
    if (!found || lifetime.first > selected_start) {
      output = transformed;
      selected_start = lifetime.first;
      found = true;
    }
  }
  return found;
}

void RelayNavigation::finish(std::uint8_t state, const std::string & reason)
{
  if (!active_ || active_->terminal) return;
  active_->terminal = true;
  active_->state = state;
  active_->reason = reason;
  trajectories_.clear();
  publish_status(active_->original.task_id, state, reason);
}

void RelayNavigation::cancel(const std::string & reason)
{
  finish(Status::CANCELED, reason);
}

void RelayNavigation::publish_status(std::uint32_t task_id, std::uint8_t state,
                                     const std::string & reason)
{
  Status status;
  status.header.stamp = node_.now();
  status.header.frame_id = map_frame_;
  status.task_id = task_id;
  status.robot_id = robot_id_;
  status.state = state;
  status.reason = reason;
  status.pose_valid = current_pose_.has_value();
  if (current_pose_) status.current_pose = *current_pose_;
  status_pub_->publish(status);
  last_status_at_ = Clock::now();
}

void RelayNavigation::trim_trajectories()
{
  while (trajectories_.size() > 32) trajectories_.erase(trajectories_.begin());
}

void RelayNavigation::tick(const std::string & flight_stage,
                           const uav_interfaces::msg::VehicleState * state, bool state_fresh,
                           FlightTask::Output & output)
{
  const auto now = Clock::now();
  const auto now_ns = node_.now().nanoseconds();
  geometry_msgs::msg::PoseStamped pose;
  if (current_pose(state, state_fresh, pose)) current_pose_ = pose;
  else current_pose_.reset();

  if (last_ros_ns_ && now_ns < *last_ros_ns_) {
    finish(Status::FAILED, "ROS 时钟回拨，请重新发送目标");
    trajectories_.clear();
    hold_.reset();
    have_goal_order_ = false;
    last_goal_token_ns_ = 0;
    last_spline_id_.reset();
    sensor_ready_ = false;
  }
  last_ros_ns_ = now_ns;

  if ((flight_stage == "SAFETY_LAND" || flight_stage == "FINISHED") && active_ && !active_->terminal)
    finish(Status::FAILED, "飞行任务已退出协同状态");

  if (active_ && !active_->terminal && !active_->dispatched) {
    if (!active_->ever_dispatched && now_ns >= active_->valid_until_ns) {
      finish(Status::FAILED, "目标在变换或起飞前过期");
    } else if (!active_->transformed) {
      geometry_msgs::msg::PoseStamped source;
      source.header = active_->original.header;
      source.pose = active_->original.target;
      active_->transformed = transform_to_map(source, active_->map_goal);
    }
  }

  if (flight_stage == "HOVER") {
    if (!hold_ && current_pose_) hold_ = current_pose_;
    const bool sensor_ok = sensors_fresh(now);
    if (active_ && !active_->terminal && active_->dispatched && !sensor_ok) {
      // A trajectory made before a depth/pose outage must not resume later.
      active_->dispatched = false;
      active_->has_association = false;
      trajectories_.clear();
      active_->state = Status::PLANNING;
      active_->reason = "等待新鲜定位与深度后重新规划";
    }
    if (active_ && !active_->terminal && active_->transformed && !active_->dispatched && !sensor_ok) {
      active_->state = Status::PLANNING;
      active_->reason = "等待新鲜定位与深度";
    }
    if (active_ && !active_->terminal && active_->transformed && !active_->dispatched &&
        current_pose_ && sensor_ok) {
      const auto token = std::max(static_cast<std::uint64_t>(now_ns), last_goal_token_ns_ + 1);
      last_goal_token_ns_ = token;
      active_->token_ns = token;
      active_->map_goal.header.stamp = rclcpp::Time(static_cast<std::int64_t>(token),
        node_.get_clock()->get_clock_type());
      planner_goal_pub_->publish(active_->map_goal);
      last_goal_sent_ = now;
      active_->dispatched = true;
      active_->ever_dispatched = true;
      active_->has_association = false;
      active_->state = Status::PLANNING;
      active_->reason = "等待本目标的有效规划";
      publish_status(active_->original.task_id, active_->state, active_->reason);
    }
    if (active_ && !active_->terminal && active_->dispatched && !active_->has_association && sensor_ok &&
        std::chrono::duration<double>(now - last_goal_sent_).count() >= 1.0) {
      planner_goal_pub_->publish(active_->map_goal);  // Same token; planner treats it as idempotent.
      last_goal_sent_ = now;
    }

    if (active_ && !active_->terminal && active_->dispatched && current_pose_) {
      const auto & p = current_pose_->pose.position;
      const auto & g = active_->map_goal.pose.position;
      if (std::hypot(std::hypot(p.x - g.x, p.y - g.y), p.z - g.z) <= arrival_tolerance_m_) {
        hold_ = current_pose_;
        finish(Status::ARRIVED, "实际位置已到达目标");
      }
    }

    geometry_msgs::msg::PoseStamped selected;
    const bool executing = current_pose_ && sensor_ok && trajectory_target(now_ns, now, selected);
    if (executing) {
      hold_ = current_pose_;
      output.target = selected;
      output.publish_target = true;
      if (active_) { active_->state = Status::EXECUTING; active_->reason = "执行本目标轨迹"; }
    } else {
      if (was_executing_ && current_pose_) hold_ = current_pose_;
      if (hold_) { output.target = *hold_; output.publish_target = true; }
      if (active_ && !active_->terminal && active_->ever_dispatched) {
        active_->state = Status::PLANNING;
        active_->reason = sensor_ok ? "等待有效轨迹" : "等待新鲜定位与深度";
      }
    }
    was_executing_ = executing;
  }

  if (last_status_at_ == Clock::time_point{} ||
      std::chrono::duration<double>(now - last_status_at_).count() >= 0.2) {
    if (active_) publish_status(active_->original.task_id, active_->state, active_->reason);
    else publish_status(0, Status::IDLE, "等待协同目标");
  }
}

}  // namespace uav_mission
