#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "uav_control/px4_dds_interface.hpp"
#ifdef BUILD_MAVROS_INTERFACE
#include "uav_control/mavros_interface.hpp"
#endif

namespace uav_control
{

class FlightControlInterface
{
public:
  explicit FlightControlInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  {
    auto config = std::make_shared<rclcpp::Node>("uav_control", options);
    const auto backend = config->declare_parameter<std::string>("backend", "");
    if (backend != "dds" && backend != "mavros") {
      throw std::invalid_argument(
        "backend must be 'dds' or 'mavros'; configure backend.yaml, got: " + backend);
    }
#ifndef BUILD_MAVROS_INTERFACE
    if (backend == "mavros") {
      throw std::runtime_error(
        "MAVROS backend is not built; rebuild with -DBUILD_MAVROS_INTERFACE=ON");
    }
#endif
    auto params_file = config->declare_parameter<std::string>("backend_params_file", "");
    if (params_file.empty()) {
      params_file = ament_index_cpp::get_package_share_directory("uav_control") +
        "/config/" + backend + ".yaml";
    }
    auto backend_options = options;
    auto arguments = options.arguments();
    arguments.insert(arguments.end(), {"--ros-args", "--params-file", params_file});
    backend_options.arguments(arguments);
    // Preserve explicit ROS/programmatic overrides above the backend file defaults.
    std::vector<rclcpp::Parameter> overrides;
    for (const auto & parameter : config->get_node_parameters_interface()->get_parameter_overrides()) {
      overrides.emplace_back(parameter.first, parameter.second);
    }
    overrides.emplace_back("backend_params_file", params_file);
    backend_options.parameter_overrides(overrides);
    config.reset();
    if (backend == "dds") {
      node_ = std::make_shared<Px4DdsInterface>(backend_options);
    }
#ifdef BUILD_MAVROS_INTERFACE
    else {
      node_ = std::make_shared<MavrosInterface>(backend_options);
    }
#endif
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    node_->declare_parameter<std::string>("backend", backend, descriptor);
    node_->declare_parameter<std::string>("backend_params_file", params_file, descriptor);
    RCLCPP_INFO(node_->get_logger(), "Selected backend: %s (fixed until restart); parameters: %s",
      backend.c_str(), params_file.c_str());
  }

  rclcpp::Node::SharedPtr get_node() const {return node_;}

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace uav_control
