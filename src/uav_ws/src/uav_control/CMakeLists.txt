cmake_minimum_required(VERSION 3.8)
project(uav_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(uav_control_node src/main.cpp src/px4_dds_interface.cpp)
target_compile_features(uav_control_node PRIVATE cxx_std_17)
target_include_directories(uav_control_node PRIVATE include)
ament_target_dependencies(uav_control_node
  rclcpp ament_index_cpp geometry_msgs px4_msgs std_srvs uav_interfaces)

option(BUILD_MAVROS_INTERFACE "Build the MAVROS control backend" ON)
if(BUILD_MAVROS_INTERFACE)
  find_package(mavros_msgs REQUIRED)
  target_sources(uav_control_node PRIVATE src/mavros_interface.cpp)
  target_compile_definitions(uav_control_node PRIVATE BUILD_MAVROS_INTERFACE)
  ament_target_dependencies(uav_control_node mavros_msgs)
endif()

install(TARGETS uav_control_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME backend_selection
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/backend_selection_test.py
      $<TARGET_FILE:uav_control_node> ${CMAKE_CURRENT_SOURCE_DIR}/config
      $<IF:$<BOOL:${BUILD_MAVROS_INTERFACE}>,ON,OFF>)
  set_tests_properties(backend_selection PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=42;ROS_LOCALHOST_ONLY=1" TIMEOUT 45)
  add_executable(dds_frames_test test/dds_frames_test.cpp)
  target_compile_features(dds_frames_test PRIVATE cxx_std_17)
  target_include_directories(dds_frames_test PRIVATE include)
  ament_target_dependencies(dds_frames_test rclcpp geometry_msgs px4_msgs std_srvs uav_interfaces)
  add_test(NAME dds_frames COMMAND dds_frames_test)
  add_executable(dds_node_test test/dds_node_test.cpp src/px4_dds_interface.cpp)
  target_compile_features(dds_node_test PRIVATE cxx_std_17)
  target_include_directories(dds_node_test PRIVATE include)
  ament_target_dependencies(dds_node_test rclcpp geometry_msgs px4_msgs std_srvs uav_interfaces)
  add_test(NAME dds_node COMMAND dds_node_test)
  set_tests_properties(dds_node PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=42;ROS_LOCALHOST_ONLY=1" TIMEOUT 45)
endif()

ament_package()
