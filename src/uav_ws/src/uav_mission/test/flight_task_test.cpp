#include <stdexcept>
#include "uav_mission/flight_task.hpp"

using Task = uav_mission::FlightTask;
using State = uav_interfaces::msg::VehicleState;
using Flight = uav_interfaces::srv::FlightCommand;

void check(bool valid, const char * message)
{
  if (!valid) {throw std::runtime_error(message);}
}

State ground_state()
{
  State state;
  state.state_valid = state.connected = state.position_valid = state.landed_state_valid = true;
  state.landed_state = State::LANDED_ON_GROUND;
  state.mode = "MANUAL";
  state.local_pose.pose.orientation.w = 1.0;
  return state;
}

int main()
{
  const auto start = Task::Clock::now();
  uav_mission::FlightTaskConfig config;
  auto state = ground_state();
  Task blocked(config);
  state.control_inhibited = true;
  auto out = blocked.update(&state, true, start);
  check(!out.publish_target && !out.send_command && blocked.stage_name() == "WAIT_READY",
    "read-only/inhibited backend allowed mission preparation");
  state.control_inhibited = false;
  out = blocked.update(&state, true, start);
  check(out.publish_target, "ready backend did not prepare target");
  state.control_inhibited = true;
  out = blocked.update(&state, true, start);
  check(out.finished && !out.passed && !out.publish_target,
    "control inhibition before ARM did not fail the task");

  state = ground_state();
  Task normal(config);
  normal.update(&state, true, start);
  normal.update(&state, true, start + std::chrono::milliseconds(1300));
  auto time = start + std::chrono::milliseconds(1400);
  out = normal.update(&state, true, time);
  check(out.send_command && out.command == Flight::Request::SET_OFFBOARD,
    "missing Offboard request");
  normal.command_dispatched(out.command_id, true, "dispatched", time);
  state.last_command_id = out.command_id;
  state.command_completed = state.command_accepted = true;
  normal.update(&state, true, time);
  check(normal.stage_name() == "REQUEST_OFFBOARD", "ACK fabricated Offboard state");
  state.mode = "OFFBOARD";
  normal.update(&state, true, time);
  out = normal.update(&state, true, time);
  check(out.send_command && out.command == Flight::Request::ARM, "missing ARM request");
  normal.command_dispatched(out.command_id, true, "dispatched", time);
  state.last_command_id = out.command_id;
  state.armed = true;
  state.landed_state = State::LANDED_IN_AIR;
  out = normal.update(&state, true, time);
  check(normal.stage_name() == "TAKEOFF" && out.target.pose.position.z == 1.0,
    "relative takeoff target incorrect");
  state.local_pose.pose.position.z = 1.0;
  normal.update(&state, true, time);
  time += std::chrono::seconds(2);
  normal.update(&state, true, time);
  check(normal.stage_name() == "HOVER", "stable period did not enter hover");
  Task interrupted = normal;
  state.control_inhibited = true;
  state.control_message = "FCU/position lost";
  out = interrupted.update(&state, true, time);
  check(interrupted.stage_name() == "SAFETY_LAND" && !out.finished && !out.publish_target,
    "airborne inhibition continued the original mission");
  out = interrupted.update(&state, true, time);
  check(out.send_command && out.command == Flight::Request::LAND,
    "airborne inhibition did not enter safety landing");
  state.control_inhibited = false;
  state.control_message.clear();
  time += std::chrono::seconds(10);
  normal.update(&state, true, time);
  out = normal.update(&state, true, time);
  check(out.send_command && out.command == Flight::Request::LAND, "missing LAND request");
  normal.command_dispatched(out.command_id, true, "dispatched", time);
  state.last_command_id = out.command_id;
  state.mode = "AUTO.LAND";
  state.control_inhibited = true;
  normal.update(&state, true, time);
  check(normal.stage_name() == "LANDING", "intentional handover treated as task failure");
  out = normal.update(&state, true, time);
  check(!out.publish_target, "AUTO.LAND received mission setpoints");
  state.landed_state = State::LANDED_ON_GROUND;
  state.armed = false;
  out = normal.update(&state, true, time);
  check(out.finished && out.passed, "ground/disarmed confirmation did not finish task");

  Task canceled(config);
  state = ground_state();
  state.armed = true;
  state.landed_state = State::LANDED_IN_AIR;
  state.mode = "OFFBOARD";
  canceled.update(&state, true, start);
  canceled.cancel(start);
  state.control_inhibited = true;
  state.control_message = "target timeout";
  out = canceled.update(&state, true, start);
  check(!out.finished && out.send_command && out.command == Flight::Request::LAND,
    "inhibition blocked safety landing");
  state.mode = "AUTO.LAND";
  state.landed_state = State::LANDED_ON_GROUND;
  state.armed = false;
  out = canceled.update(&state, true, start);
  check(out.finished && out.canceled && !out.passed, "cancellation reported flight success");
}
