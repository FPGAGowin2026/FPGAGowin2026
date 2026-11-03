#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <swarm_interfaces/msg/relay_goal.hpp>
#include <swarm_interfaces/msg/relay_status.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <traj_utils/msg/bspline.hpp>

#include "uav_mission/flight_task.hpp"

namespace uav_mission
{

// One mission node selects either the flight task's hold target or the EGO
// trajectory; this helper never publishes directly to uav_control or MAVROS.
class RelayNavigation
{
#ifdef UAV_RELAY_TEST
  friend class RelayNavigationHarness;
#endif
public:
  explicit RelayNavigation(rclcpp::Node & node);
  void tick(const std::string & flight_stage,
            const uav_interfaces::msg::VehicleState * state, bool state_fresh,
            FlightTask::Output & output);
  void cancel(const std::string & reason);

private:
  using Clock = std::chrono::steady_clock;
  using Goal = swarm_interfaces::msg::RelayGoal;
  using Status = swarm_interfaces::msg::RelayStatus;
  using Command = quadrotor_msgs::msg::PositionCommand;
  using Bspline = traj_utils::msg::Bspline;

  struct ActiveGoal
  {
    Goal original;
    std::int64_t source_ns{0};
    std::int64_t valid_until_ns{0};
    geometry_msgs::msg::PoseStamped map_goal;
    bool transformed{false};
    bool dispatched{false};
    bool ever_dispatched{false};
    bool has_association{false};
    bool terminal{false};
    std::uint64_t token_ns{0};
    std::uint8_t state{Status::ACCEPTED};
    std::string reason;
  };

  struct Trajectory
  {
    std::optional<std::uint64_t> goal_token;
    std::optional<std::pair<std::int64_t, std::int64_t>> lifetime;
    std::optional<Command> command;
    std::int64_t command_stamp_ns{0};
    Clock::time_point command_received{};
  };

  void on_goal(const Goal::SharedPtr message);
  void on_association(const std_msgs::msg::UInt64MultiArray::SharedPtr message);
  void on_spline(const Bspline::SharedPtr message);
  void on_command(const Command::SharedPtr message);
  void on_sensor_ready(const std_msgs::msg::Bool::SharedPtr message);
  bool transform_to_map(const geometry_msgs::msg::PoseStamped & input,
                        geometry_msgs::msg::PoseStamped & output) const;
  bool current_pose(const uav_interfaces::msg::VehicleState * state, bool state_fresh,
                    geometry_msgs::msg::PoseStamped & output) const;
  bool sensors_fresh(Clock::time_point now) const;
  bool trajectory_target(std::int64_t now_ns, Clock::time_point now,
                         geometry_msgs::msg::PoseStamped & output) const;
  void finish(std::uint8_t state, const std::string & reason);
  void publish_status(std::uint32_t task_id, std::uint8_t state, const std::string & reason);
  void trim_trajectories();

  rclcpp::Node & node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string robot_id_;
  std::string map_frame_;
  double arrival_tolerance_m_{0.25};
  double command_timeout_s_{0.3};
  double sensor_timeout_s_{0.5};

  std::optional<ActiveGoal> active_;
  std::map<std::uint32_t, Trajectory> trajectories_;
  std::optional<geometry_msgs::msg::PoseStamped> hold_;
  std::optional<geometry_msgs::msg::PoseStamped> current_pose_;
  Clock::time_point sensor_received_{};
  Clock::time_point last_goal_sent_{};
  Clock::time_point last_status_at_{};
  bool sensor_ready_{false};
  bool was_executing_{false};
  bool have_goal_order_{false};
  std::int64_t last_goal_source_ns_{0};
  std::uint32_t last_goal_id_{0};
  std::uint64_t last_goal_token_ns_{0};
  std::optional<std::int64_t> last_ros_ns_;
  std::optional<std::uint32_t> last_spline_id_;
  std::int64_t last_spline_start_ns_{0};

  rclcpp::Subscription<Goal>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr association_sub_;
  rclcpp::Subscription<Bspline>::SharedPtr spline_sub_;
  rclcpp::Subscription<Command>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensor_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr planner_goal_pub_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
};

}  // namespace uav_mission
