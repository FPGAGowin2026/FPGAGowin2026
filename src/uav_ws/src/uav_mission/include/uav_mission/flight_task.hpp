#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>
#include <uav_interfaces/srv/flight_command.hpp>

namespace uav_mission
{

struct FlightTaskConfig
{
  double takeoff_height_m{1.0};
  double horizontal_tolerance_m{0.2};
  double vertical_tolerance_m{0.15};
  double stable_duration_s{2.0};
  double hover_duration_s{10.0};
  double ready_timeout_s{30.0};
  double command_timeout_s{5.0};
  double takeoff_timeout_s{30.0};
  double landing_timeout_s{45.0};
  bool relay_mode{false};
};

class FlightTask
{
public:
  using Clock = std::chrono::steady_clock;

  struct Output
  {
    bool publish_target{false};
    geometry_msgs::msg::PoseStamped target;
    bool send_command{false};
    uint32_t command_id{0};
    uint8_t command{0};
    bool finished{false};
    bool passed{false};
    bool canceled{false};
  };

  explicit FlightTask(const FlightTaskConfig & config);
  Output update(
    const uav_interfaces::msg::VehicleState * state,
    bool state_stream_fresh, Clock::time_point now);
  void command_dispatched(
    uint32_t command_id, bool dispatched, const std::string & message, Clock::time_point now);
  void cancel(Clock::time_point now);

  const std::string & stage_name() const;
  const std::string & result_reason() const;
  double max_horizontal_error_m() const;
  double max_vertical_error_m() const;
  double max_yaw_error_rad() const;
  double final_horizontal_displacement_m() const;
  double final_vertical_displacement_m() const;
  bool have_hover_measurements() const;
  double last_horizontal_target_error_m() const;
  double last_vertical_target_error_m() const;
  bool have_position_measurement() const;

private:
  enum class Stage
  {
    WAIT_READY,
    PREPARE_OFFBOARD,
    REQUEST_OFFBOARD,
    REQUEST_ARM,
    TAKEOFF,
    STABILIZE,
    HOVER,
    REQUEST_LAND,
    REQUEST_DISARM,
    LANDING,
    SAFETY_LAND,
    FINISHED
  };

  void enter(Stage stage, Clock::time_point now);
  void begin_failure(const std::string & reason, Clock::time_point now);
  void begin_safety(Clock::time_point now);
  void finish(bool passed, bool canceled);
  void request_command(uint8_t command);
  bool command_accepted(const uav_interfaces::msg::VehicleState * state) const;
  bool command_rejected(const uav_interfaces::msg::VehicleState * state) const;
  bool status_valid(
    const uav_interfaces::msg::VehicleState * state, bool state_stream_fresh) const;
  bool flight_state_valid(
    const uav_interfaces::msg::VehicleState * state, bool state_stream_fresh) const;
  void update_errors(const uav_interfaces::msg::VehicleState & state);
  void set_target(double x, double y, double z, double yaw);
  double horizontal_error(const uav_interfaces::msg::VehicleState & state) const;
  double vertical_error(const uav_interfaces::msg::VehicleState & state) const;
  double yaw_error(const uav_interfaces::msg::VehicleState & state) const;
  Output output(Clock::time_point now);

  FlightTaskConfig config_;
  Stage stage_{Stage::WAIT_READY};
  Clock::time_point stage_started_{};
  Clock::time_point command_started_{};
  Clock::time_point takeoff_started_{};
  Clock::time_point landing_started_{};
  Clock::time_point stable_since_{};
  Clock::time_point safety_retry_after_{};
  bool have_stable_since_{false};
  bool flight_may_be_armed_{false};
  bool last_state_fresh_{false};
  uav_interfaces::msg::VehicleState last_state_;
  bool hover_measurements_valid_{false};
  bool have_position_measurement_{false};
  bool target_valid_{false};
  bool command_request_sent_{false};
  bool command_dispatched_{false};
  bool command_dispatched_ok_{false};
  bool cancel_requested_{false};
  bool finished_{false};
  bool passed_{false};
  bool canceled_{false};
  uint32_t next_command_id_{1};
  uint32_t pending_command_id_{0};
  uint8_t pending_command_{0};
  std::string command_dispatch_message_;
  std::string failure_reason_;
  std::string stage_name_;
  geometry_msgs::msg::PoseStamped target_;
  double home_x_{0.0};
  double home_y_{0.0};
  double home_z_{0.0};
  double home_yaw_{0.0};
  double target_z_{0.0};
  double max_horizontal_error_m_{0.0};
  double max_vertical_error_m_{0.0};
  double max_yaw_error_rad_{0.0};
  double final_horizontal_displacement_m_{0.0};
  double final_vertical_displacement_m_{0.0};
  double last_horizontal_target_error_m_{0.0};
  double last_vertical_target_error_m_{0.0};
};

}  // namespace uav_mission
