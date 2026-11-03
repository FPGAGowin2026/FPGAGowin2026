// In-process synthetic PX4 checks. All interfaces remapped; no Agent or aircraft.
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>
#include "uav_control/px4_dds_interface.hpp"

using Status = px4_msgs::msg::VehicleStatus;
using Command = px4_msgs::msg::VehicleCommand;
using Ack = px4_msgs::msg::VehicleCommandAck;
using State = uav_interfaces::msg::VehicleState;
using Flight = uav_interfaces::srv::FlightCommand;

void check(bool ok, const char * reason)
{
  if (!ok) {throw std::runtime_error(reason);}
}

void run(bool read_only, bool handover_only = false)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("read_only", read_only),
    rclcpp::Parameter("fmu_prefix", "/dds_test/fmu"),
    rclcpp::Parameter("command_timeout_s", 0.6)});
  options.arguments({"--ros-args", "-r", "/uav_control/state:=/dds_test/state",
    "-r", "/uav_control/target:=/dds_test/target", "-r", "/uav_control/command:=/dds_test/command",
    "-r", "/uav_control/reset:=/dds_test/reset"});
  auto control = std::make_shared<uav_control::Px4DdsInterface>(options);
  auto mock = std::make_shared<rclcpp::Node>("dds_test_mock");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(control);
  executor.add_node(mock);
  State state;
  bool have_state = false;
  std::vector<px4_msgs::msg::TrajectorySetpoint> setpoints;
  std::vector<Command> commands;
  size_t heartbeats = 0;
  auto state_sub = mock->create_subscription<State>("/dds_test/state", 10,
    [&](State::SharedPtr msg) {state = *msg; have_state = true;});
  auto setpoint_sub = mock->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
    "/dds_test/fmu/in/trajectory_setpoint", 10,
    [&](px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {setpoints.push_back(*msg);});
  auto command_sub = mock->create_subscription<Command>("/dds_test/fmu/in/vehicle_command", 10,
    [&](Command::SharedPtr msg) {commands.push_back(*msg);});
  auto heartbeat_sub = mock->create_subscription<px4_msgs::msg::OffboardControlMode>(
    "/dds_test/fmu/in/offboard_control_mode", 10,
    [&](px4_msgs::msg::OffboardControlMode::SharedPtr) {++heartbeats;});
  const auto qos = rclcpp::SensorDataQoS();
  auto status_pub = mock->create_publisher<Status>("/dds_test/fmu/out/vehicle_status_v1", qos);
  auto position_pub = mock->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
    "/dds_test/fmu/out/vehicle_local_position_v1", qos);
  auto attitude_pub = mock->create_publisher<px4_msgs::msg::VehicleAttitude>(
    "/dds_test/fmu/out/vehicle_attitude", qos);
  auto land_pub = mock->create_publisher<px4_msgs::msg::VehicleLandDetected>(
    "/dds_test/fmu/out/vehicle_land_detected", qos);
  auto sync_pub = mock->create_publisher<px4_msgs::msg::TimesyncStatus>(
    "/dds_test/fmu/out/timesync_status", qos);
  auto ack_pub = mock->create_publisher<Ack>("/dds_test/fmu/out/vehicle_command_ack", qos);
  auto target_pub = mock->create_publisher<geometry_msgs::msg::PoseStamped>("/dds_test/target", 10);
  auto command_client = mock->create_client<Flight>("/dds_test/command");
  auto reset_client = mock->create_client<std_srvs::srv::Trigger>("/dds_test/reset");
  Status status;
  status.system_id = status.component_id = 1;
  status.arming_state = Status::ARMING_STATE_DISARMED;
  px4_msgs::msg::VehicleLocalPosition position;
  position.x = 2; position.y = -3; position.z = -4;
  position.xy_valid = position.z_valid = position.heading_good_for_control = true;
  px4_msgs::msg::VehicleAttitude attitude;
  attitude.q = {1, 0, 0, 0};
  px4_msgs::msg::VehicleLandDetected land;
  land.landed = true;
  px4_msgs::msg::TimesyncStatus sync;
  sync.source_protocol = sync.SOURCE_PROTOCOL_DDS;
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = "map";
  target.pose.position.x = 1; target.pose.position.y = 2; target.pose.position.z = 3;
  target.pose.orientation.z = std::sin(0.3); target.pose.orientation.w = std::cos(0.3);
  double sync_age = 0.0;
  auto pump = [&](double seconds, bool send_target = false, bool stream = true, bool frozen = false) {
      const auto start = std::chrono::steady_clock::now();
      while (std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count() < seconds) {
        if (!frozen) {
          const auto stamp = mock->now().nanoseconds() / 1000;
          status.timestamp = position.timestamp = attitude.timestamp = land.timestamp =
            sync.timestamp = sync.remote_timestamp = stamp;
          position.timestamp_sample = attitude.timestamp_sample = stamp;
          sync.timestamp -= static_cast<uint64_t>(sync_age * 1e6);
          sync.remote_timestamp = sync.timestamp;
        }
        if (stream) {
          status_pub->publish(status); position_pub->publish(position); attitude_pub->publish(attitude);
          land_pub->publish(land); sync_pub->publish(sync);
        }
        if (send_target) {target.header.stamp = mock->now(); target_pub->publish(target);}
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      executor.spin_some();
    };
  auto command = [&](uint32_t id, uint8_t code) {
      auto req = std::make_shared<Flight::Request>(); req->command_id = id; req->command = code;
      auto future = command_client->async_send_request(req);
      for (int i = 0; i < 50 && future.wait_for(std::chrono::seconds(0)) != std::future_status::ready;
        ++i) {pump(0.01, true);}
      check(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready, "service timeout");
      const auto response = future.get();
      if (!response->dispatched) {
        std::cerr << "command id=" << id << " rejected: " << response->message << '\n';
      }
      return response->dispatched;
    };
  auto ack = [&](uint8_t result, uint16_t component = 191, uint64_t stamp = 0) {
      Ack msg;
      msg.timestamp = stamp ? stamp : mock->now().nanoseconds() / 1000;
      msg.command = commands.back().command;
      msg.target_system = 1; msg.target_component = component; msg.result = result;
      ack_pub->publish(msg); pump(0.08, true);
    };
  auto reset = [&]() {
      auto future = reset_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      for (int i = 0; i < 50 && future.wait_for(std::chrono::seconds(0)) != std::future_status::ready;
        ++i) {pump(0.01);}
      check(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready, "reset timeout");
      return future.get()->success;
    };
  auto wait_state = [&](const auto & predicate, const char * reason) {
      for (int i = 0; i < 30 && !predicate(); ++i) {pump(0.02, true);}
      check(predicate(), reason);
    };
  const auto ready_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!(have_state && state.state_valid && state.position_valid) &&
    std::chrono::steady_clock::now() < ready_deadline)
  {
    pump(0.02);
  }
  check(have_state && state.state_valid && state.position_valid, "fresh state/position not mapped");
  check(state.local_pose.pose.position.x == -3 && state.local_pose.pose.position.y == 2 &&
    state.local_pose.pose.position.z == 4, "state NED to ENU");
  check(std::abs(state.local_pose.pose.orientation.z - std::sqrt(0.5)) < 1e-6, "attitude mapping");
  check(heartbeats == 0 && setpoints.empty(), "published without a target");
  if (read_only) {
    check(state.control_inhibited, "readonly did not inhibit missions");
    sync_age = 1.5; pump(3.2);
    check(state.connected, "timesync was incorrectly limited by state_timeout_s");
    sync_age = 4.0; pump(3.2);
    check(!state.connected, "expired timesync kept FCU state valid");
    sync_age = 0.0; pump(0.2);
    pump(0.3, true);
    check(heartbeats == 0 && commands.empty(), "readonly created control traffic");
    check(!command_client->service_is_ready(), "readonly exposes command service");
    pump(1.2, false, true, true);
    check(!state.connected && !state.position_valid, "frozen timestamps refreshed state");
    pump(0.2);
    check(state.connected, "readonly failed to resume data");
    position.xy_valid = false; pump(0.1);
    check(!state.position_valid, "position validity flag ignored");
    std::cout << "PASS: readonly, state mapping, stream timeouts, frozen timestamps, reconnect\n";
    return;
  }
  target.header.frame_id = "wrong"; pump(0.2, true);
  check(setpoints.empty(), "invalid frame accepted");
  target.header.frame_id = "map"; pump(0.2, true);
  if (handover_only) {
    pump(1.2, true);
    status.arming_state = Status::ARMING_STATE_ARMED;
    status.nav_state = Status::NAVIGATION_STATE_OFFBOARD; land.landed = false;
    wait_state([&]() {
      return state.state_valid && state.position_valid && state.armed &&
             state.mode == "OFFBOARD" && !state.control_inhibited;
    }, "synthetic OFFBOARD/armed state not ready for LAND");
    check(command(1, Flight::Request::LAND), "LAND dispatch failed");
    ack(Ack::VEHICLE_CMD_RESULT_ACCEPTED);
    check(state.command_accepted && state.mode == "OFFBOARD", "LAND ACK fabricated mode");
    auto active = heartbeats; pump(0.1, true);
    check(heartbeats > active, "LAND stream stopped before mode handover");
    status.nav_state = Status::NAVIGATION_STATE_AUTO_LAND; pump(0.2, true);
    wait_state([&]() {
      return state.mode == "AUTO.LAND" && state.control_inhibited && state.position_valid;
    }, "LAND handover did not distinguish control inhibition from physical position");
    auto stopped = heartbeats; pump(0.2, true);
    check(heartbeats == stopped, "AUTO.LAND did not stop heartbeat");
    land.landed = true; pump(0.1);
    check(command(2, Flight::Request::DISARM), "ground DISARM dispatch failed");
    ack(Ack::VEHICLE_CMD_RESULT_ACCEPTED);
    check(state.command_accepted && state.armed, "DISARM ACK fabricated arming state");
    status.arming_state = Status::ARMING_STATE_DISARMED;
    status.nav_state = Status::NAVIGATION_STATE_MANUAL;
    pump(0.1);
    check(reset(), "post-LAND reset failed");
    wait_state([&]() {return !state.control_inhibited;}, "ground reset retained inhibition");
    pump(1.4, true);
    status.nav_state = Status::NAVIGATION_STATE_OFFBOARD; pump(0.1, true);
    status.nav_state = Status::NAVIGATION_STATE_POSCTL; pump(0.2, true);
    stopped = heartbeats; pump(0.2, true);
    check(state.control_inhibited && heartbeats == stopped,
      "unexpected mode takeover retained Offboard traffic");
    status.nav_state = Status::NAVIGATION_STATE_OFFBOARD; pump(0.2, true);
    check(heartbeats == stopped, "mode recovery resumed old task");
    status.nav_state = Status::NAVIGATION_STATE_MANUAL; pump(0.1);
    check(reset(), "takeover ground reset failed");
    pump(1.4, true);
    status.nav_state = Status::NAVIGATION_STATE_OFFBOARD; pump(0.1, true);
    check(command(1, Flight::Request::ARM), "ARM before localization loss failed");
    position.xy_valid = false; pump(0.1, true);
    check(state.control_inhibited && state.command_completed && !state.command_accepted,
      "localization loss retained pending command");
    position.xy_valid = true; pump(0.1, true);
    ack(Ack::VEHICLE_CMD_RESULT_ACCEPTED);
    check(!state.command_accepted && state.position_valid && state.control_inhibited,
      "recovered localization accepted delayed ACK or cleared inhibition");
    std::cout << "PASS: LAND handover, DISARM ACK/state separation, takeover, pending command loss\n";
    return;
  }
  check(!command(1, Flight::Request::SET_OFFBOARD), "offboard before prestream");
  pump(1.2, true);
  check(setpoints.size() > 20 && heartbeats > 20, "no continuous target stream");
  const auto & sp = setpoints.back();
  check(sp.position == (std::array<float, 3>{2, 1, -3}), "target ENU to NED");
  check(std::abs(sp.yaw - (uav_control::dds_frames::pi/2 - 0.6)) < 1e-6, "target yaw conversion");
  for (float v : sp.velocity) {check(std::isnan(v), "velocity not NaN");}
  for (float v : sp.acceleration) {check(std::isnan(v), "acceleration not NaN");}
  check(std::isnan(sp.yawspeed), "yawspeed not NaN");
  check(command(1, Flight::Request::SET_OFFBOARD), "offboard dispatch failed");
  check(!command(2, Flight::Request::ARM), "concurrent command accepted");
  ack(Ack::VEHICLE_CMD_RESULT_ACCEPTED, 192);
  check(!state.command_completed, "wrong ACK route accepted");
  ack(Ack::VEHICLE_CMD_RESULT_ACCEPTED, 191, commands.back().timestamp - 1);
  check(!state.command_completed, "old ACK accepted");
  ack(Ack::VEHICLE_CMD_RESULT_IN_PROGRESS);
  check(!state.command_completed, "in-progress ACK completed command");
  ack(Ack::VEHICLE_CMD_RESULT_ACCEPTED);
  check(state.command_completed && state.command_accepted && state.mode == "MANUAL",
    "ACK/state separation failed");
  status.nav_state = Status::NAVIGATION_STATE_OFFBOARD; pump(0.1, true);
  check(command(2, Flight::Request::ARM), "arm dispatch failed");
  pump(0.7, true);
  check(state.last_command_id == 2 && state.command_completed && !state.command_accepted,
    "command timeout not reported");
  ack(Ack::VEHICLE_CMD_RESULT_ACCEPTED);
  check(!state.command_accepted && !command(3, Flight::Request::ARM), "late ACK/retry not quarantined");
  check(command(3, Flight::Request::SET_OFFBOARD), "second mode dispatch failed");
  ack(Ack::VEHICLE_CMD_RESULT_DENIED);
  check(state.command_completed && !state.command_accepted, "denied ACK not reported");
  status.arming_state = Status::ARMING_STATE_ARMED; land.landed = false;
  pump(1.2);
  check(commands.back().command == Command::VEHICLE_CMD_NAV_LAND, "target loss did not dispatch LAND");
  check(state.control_inhibited, "target timeout did not inhibit the mission");
  pump(0.2); auto stopped = heartbeats; pump(0.2);
  check(heartbeats == stopped, "old target maintained heartbeat");
  check(!command(4, Flight::Request::DISARM), "airborne disarm allowed");
  status.nav_state = Status::NAVIGATION_STATE_AUTO_LAND; pump(0.1);
  check(state.mode == "AUTO.LAND", "land mode not mapped");
  land.landed = true; status.arming_state = Status::ARMING_STATE_DISARMED;
  status.nav_state = Status::NAVIGATION_STATE_MANUAL; pump(0.7);
  check(reset(), "ground reset failed");
  pump(0.2); check(heartbeats == stopped, "reset reused old target");
  pump(0.3, true); ++position.xy_reset_counter; pump(0.2, true);
  check(!state.position_valid, "origin reset not latched");
  stopped = heartbeats; pump(0.2, true);
  check(heartbeats == stopped, "origin reset retained heartbeat");
  check(reset(), "origin reset recovery failed");
  pump(0.3, true); pump(1.2, false, false);
  check(!state.connected, "link loss retained connected state");
  stopped = heartbeats; pump(0.3, true);
  check(state.connected && heartbeats == stopped, "reconnect resumed old task");
  check(state.control_inhibited, "reconnect silently cleared control inhibition");
  std::cout << "PASS: targets, prestream, ACK routing/progress/rejection/timeout, target loss, resets\n";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {run(true); run(false); run(false, true);} catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; result = 1;
  }
  rclcpp::shutdown();
  return result;
}
