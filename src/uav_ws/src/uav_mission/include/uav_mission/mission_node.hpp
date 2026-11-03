#pragma once

#include <chrono>
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>
#include <uav_interfaces/srv/flight_command.hpp>

#include "uav_mission/flight_task.hpp"
#ifdef UAV_RELAY_ENABLED
#include "uav_mission/relay_navigation.hpp"
#endif

namespace uav_mission
{

class MissionNode : public rclcpp::Node
{
public:
  MissionNode();

private:
  using Clock = FlightTask::Clock;
  void state_callback(const uav_interfaces::msg::VehicleState::SharedPtr msg);
  void cancel_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void timer_callback();
  void report_result(const FlightTask::Output & output);

  std::unique_ptr<FlightTask> task_;
#ifdef UAV_RELAY_ENABLED
  std::unique_ptr<RelayNavigation> relay_;
#endif
  std::optional<uav_interfaces::msg::VehicleState> latest_state_;
  Clock::time_point state_rx_{};
  double state_timeout_s_{};
  bool result_reported_{false};
  std::string last_logged_stage_;

  rclcpp::Subscription<uav_interfaces::msg::VehicleState>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Client<uav_interfaces::srv::FlightCommand>::SharedPtr command_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uav_mission
