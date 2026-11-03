#include "uav_mission/relay_navigation.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

#define REQUIRE(condition) do { if (!(condition)) throw std::runtime_error(#condition); } while (0)

namespace uav_mission
{

void check_landed_state_only_when_needed()
{
  FlightTaskConfig config;
  config.stable_duration_s = 0.1;
  config.hover_duration_s = 0.1;
  FlightTask task(config);
  auto now = FlightTask::Clock::now();
  uav_interfaces::msg::VehicleState state;
  state.state_valid = state.connected = state.position_valid = state.landed_state_valid = true;
  state.landed_state = uav_interfaces::msg::VehicleState::LANDED_ON_GROUND;
  state.local_pose.pose.orientation.w = 1.0;
  state.mode = "MANUAL";
  task.update(&state, true, now);
  now += std::chrono::seconds(2);
  task.update(&state, true, now);
  now += std::chrono::milliseconds(10);
  auto command = task.update(&state, true, now);
  REQUIRE(command.send_command);
  task.command_dispatched(command.command_id, true, "accepted", now);
  state.last_command_id = command.command_id;
  state.command_completed = state.command_accepted = true;
  state.mode = "OFFBOARD";
  task.update(&state, true, now);
  now += std::chrono::milliseconds(10);
  command = task.update(&state, true, now);
  REQUIRE(command.send_command);
  task.command_dispatched(command.command_id, true, "accepted", now);
  state.last_command_id = command.command_id;
  state.armed = true;
  task.update(&state, true, now);
  state.local_pose.pose.position.z = 1.0;
  now += std::chrono::milliseconds(10);
  task.update(&state, true, now);
  now += std::chrono::milliseconds(110);
  task.update(&state, true, now);
  REQUIRE(task.stage_name() == "HOVER");

  state.landed_state_valid = false;
  task.update(&state, true, now);
  REQUIRE(task.stage_name() == "HOVER");
  now += std::chrono::milliseconds(110);
  task.update(&state, true, now);
  REQUIRE(task.stage_name() == "REQUEST_LAND");
  command = task.update(&state, true, now);
  REQUIRE(command.send_command);
  task.command_dispatched(command.command_id, true, "accepted", now);
  state.last_command_id = command.command_id;
  state.mode = "AUTO.LAND";
  task.update(&state, true, now);
  REQUIRE(task.stage_name() == "LANDING");
  state.position_valid = false;
  REQUIRE(!task.update(&state, true, now).finished);
  REQUIRE(task.stage_name() == "LANDING");
  state.landed_state_valid = true;
  task.update(&state, true, now);
  REQUIRE(task.stage_name() == "REQUEST_DISARM");
  state.landed_state_valid = false;
  state.mode = "MANUAL";
  REQUIRE(!task.update(&state, true, now).finished);
  REQUIRE(task.stage_name() == "REQUEST_DISARM");
  state.armed = false;
  state.landed_state_valid = true;
  REQUIRE(task.update(&state, true, now).finished);
}

class RelayNavigationHarness
{
  using Goal = swarm_interfaces::msg::RelayGoal;
  using Status = swarm_interfaces::msg::RelayStatus;
  using Command = quadrotor_msgs::msg::PositionCommand;
  using Bspline = traj_utils::msg::Bspline;

  static Goal::SharedPtr goal(rclcpp::Node & node, std::uint32_t id, double x)
  {
    auto result = std::make_shared<Goal>();
    result->header.stamp = node.now();
    result->header.frame_id = "source";
    result->task_id = id;
    result->robot_id = "uav_01";
    result->target.position.x = x;
    result->target.position.z = 1.0;
    result->target.orientation.w = 1.0;
    result->valid_for.sec = 60;
    return result;
  }

  static Bspline::SharedPtr spline(rclcpp::Node & node, std::int64_t id, double duration)
  {
    auto result = std::make_shared<Bspline>();
    result->traj_id = id;
    result->order = 3;
    result->start_time = node.now();
    result->pos_pts.resize(4);
    result->knots = {-0.3, -0.2, -0.1, 0.0, duration,
                     duration + 0.1, duration + 0.2, duration + 0.3};
    return result;
  }

  static Command::SharedPtr command(rclcpp::Node & node, std::uint32_t id, double x)
  {
    auto result = std::make_shared<Command>();
    result->header.frame_id = "map";
    result->header.stamp = node.now();
    result->trajectory_id = id;
    result->trajectory_flag = Command::TRAJECTORY_STATUS_READY;
    result->position.x = x;
    result->position.z = 1.0;
    return result;
  }

  static void sensors(RelayNavigation & relay, bool ready)
  {
    auto flag = std::make_shared<std_msgs::msg::Bool>();
    flag->data = ready;
    relay.on_sensor_ready(flag);
  }

 public:
  static void run(rclcpp::Node & node)
  {
    RelayNavigation relay(node);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "source";
    transform.header.stamp = node.now();
    transform.transform.translation.x = 1.0;
    transform.transform.rotation.w = 1.0;
    REQUIRE(relay.tf_buffer_.setTransform(transform, "test", true));

    uav_interfaces::msg::VehicleState state;
    state.position_valid = true;
    state.local_pose.header.frame_id = "map";
    state.local_pose.header.stamp = node.now();
    state.local_pose.pose.position.z = 1.0;
    state.local_pose.pose.orientation.w = 1.0;
    FlightTask::Output output;
    output.publish_target = true;
    output.target = state.local_pose;

    const auto first_goal = goal(node, 1, 1.0);
    relay.on_goal(first_goal);
    relay.tick("TAKEOFF", &state, true, output);
    REQUIRE(relay.active_ && !relay.active_->dispatched);
    sensors(relay, true);
    relay.tick("HOVER", &state, true, output);
    REQUIRE(relay.active_->dispatched && relay.active_->map_goal.pose.position.x == 2.0);
    const auto first_token = relay.active_->token_ns;
    relay.on_goal(first_goal);  // Replayed goal is idempotent, not a new generation.
    REQUIRE(relay.active_->token_ns == first_token);

    const auto first_spline = spline(node, 1, 1.0);
    const auto first_command = command(node, 1, 0.5);
    relay.on_command(first_command);
    relay.on_spline(first_spline);
    relay.last_goal_sent_ -= std::chrono::seconds(2);
    const auto before_retry = relay.last_goal_sent_;
    relay.tick("HOVER", &state, true, output);
    REQUIRE(relay.active_->state == Status::PLANNING && output.target.pose.position.x == 0.0 &&
            relay.active_->token_ns == first_token && relay.last_goal_sent_ > before_retry &&
            relay.trajectories_.at(1).lifetime && relay.trajectories_.at(1).command);
    auto association = std::make_shared<std_msgs::msg::UInt64MultiArray>();
    association->data = {first_token, 1};
    relay.on_association(association);
    relay.tick("HOVER", &state, true, output);
    REQUIRE(relay.active_->state == Status::EXECUTING && output.target.pose.position.x == 0.5);

    sensors(relay, false);
    relay.tick("HOVER", &state, true, output);
    REQUIRE(!relay.active_->dispatched && output.target.pose.position.x == 0.0);
    sensors(relay, true);
    relay.tick("HOVER", &state, true, output);
    REQUIRE(relay.active_->dispatched && relay.active_->token_ns != first_token);
    const auto second_token = relay.active_->token_ns;
    relay.on_association(association);  // Old goal association must not regain control.
    relay.on_spline(first_spline);
    relay.on_command(first_command);
    relay.tick("HOVER", &state, true, output);
    REQUIRE(output.target.pose.position.x == 0.0);

    const auto second_spline = spline(node, 2, 0.5);
    const auto second_command = command(node, 2, 0.75);
    association->data = {second_token, 2};
    relay.on_association(association);
    relay.on_spline(second_spline);
    relay.on_command(second_command);
    relay.tick("HOVER", &state, true, output);
    REQUIRE(relay.active_->state == Status::EXECUTING && output.target.pose.position.x == 0.75);
    std::this_thread::sleep_for(std::chrono::milliseconds(550));
    sensors(relay, true);
    relay.on_command(command(node, 2, 0.75));  // Fresh command cannot extend an expired spline.
    relay.tick("HOVER", &state, true, output);
    REQUIRE(relay.active_->state == Status::PLANNING && output.target.pose.position.x == 0.0);

    state.local_pose.pose.position.x = 2.0;
    relay.tick("HOVER", &state, true, output);
    REQUIRE(relay.active_->terminal && relay.active_->state == Status::ARRIVED);

    auto expired = goal(node, 2, 3.0);
    expired->header.stamp = rclcpp::Time(node.now().nanoseconds() - 2000000000LL);
    expired->valid_for.sec = 1;
    relay.on_goal(expired);
    REQUIRE(relay.active_->terminal && relay.active_->original.task_id == 1);

    // A restarted gateway may reuse task ID 1, including small clock skew.
    auto after_restart = goal(node, 1, 3.0);
    after_restart->header.stamp = rclcpp::Time(node.now().nanoseconds() + 20000000LL);
    relay.on_goal(after_restart);
    REQUIRE(relay.active_ && !relay.active_->terminal && relay.active_->original.task_id == 1);
    relay.tick("HOVER", &state, true, output);
    REQUIRE(relay.active_->dispatched);
    const auto newer_spline = spline(node, 10, 1.0);
    relay.on_spline(newer_spline);
    auto restarted = spline(node, 1, 1.0);
    relay.on_spline(restarted);
    REQUIRE(relay.active_->terminal && relay.active_->state == Status::FAILED);
  }
};

}  // namespace uav_mission

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rclcpp::Node>("relay_navigation_test");
    uav_mission::check_landed_state_only_when_needed();
    uav_mission::RelayNavigationHarness::run(*node);
    node.reset();
    rclcpp::shutdown();
    std::cout << "Relay navigation checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    rclcpp::shutdown();
    std::cerr << "Relay navigation check failed: " << error.what() << '\n';
    return 1;
  }
}
