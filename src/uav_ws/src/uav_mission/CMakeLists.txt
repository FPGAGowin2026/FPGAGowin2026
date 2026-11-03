cmake_minimum_required(VERSION 3.8)
project(uav_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(std_msgs QUIET)
find_package(tf2 QUIET)
find_package(tf2_ros QUIET)
find_package(tf2_geometry_msgs QUIET)
find_package(swarm_interfaces QUIET)
find_package(quadrotor_msgs QUIET)
find_package(traj_utils QUIET)

add_executable(mission_node
  src/flight_task.cpp
  src/mission_node.cpp
)
target_compile_features(mission_node PRIVATE cxx_std_17)
target_include_directories(mission_node PRIVATE include)
ament_target_dependencies(mission_node
  geometry_msgs
  rclcpp
  std_srvs
  uav_interfaces
)

install(TARGETS mission_node DESTINATION lib/${PROJECT_NAME})
if(std_msgs_FOUND AND tf2_FOUND AND tf2_ros_FOUND AND tf2_geometry_msgs_FOUND AND
   swarm_interfaces_FOUND AND quadrotor_msgs_FOUND AND traj_utils_FOUND)
  add_executable(relay_mission_node
    src/flight_task.cpp
    src/mission_node.cpp
    src/relay_navigation.cpp
  )
  target_compile_features(relay_mission_node PRIVATE cxx_std_17)
  target_compile_definitions(relay_mission_node PRIVATE UAV_RELAY_ENABLED)
  target_include_directories(relay_mission_node PRIVATE include)
  ament_target_dependencies(relay_mission_node
    geometry_msgs rclcpp std_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs
    uav_interfaces swarm_interfaces quadrotor_msgs traj_utils)
  install(TARGETS relay_mission_node DESTINATION lib/${PROJECT_NAME})
  if(BUILD_TESTING)
    enable_testing()
    add_executable(test_relay_navigation test/test_relay_navigation.cpp src/relay_navigation.cpp src/flight_task.cpp)
    target_compile_features(test_relay_navigation PRIVATE cxx_std_17)
    target_compile_definitions(test_relay_navigation PRIVATE UAV_RELAY_TEST)
    target_include_directories(test_relay_navigation PRIVATE include)
    ament_target_dependencies(test_relay_navigation
      geometry_msgs rclcpp std_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs
      uav_interfaces swarm_interfaces quadrotor_msgs traj_utils)
    add_test(NAME test_relay_navigation COMMAND test_relay_navigation)
  endif()
else()
  message(STATUS "Relay mission dependencies unavailable; building the original mission only")
endif()
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(flight_task_test test/flight_task_test.cpp src/flight_task.cpp)
  target_compile_features(flight_task_test PRIVATE cxx_std_17)
  target_include_directories(flight_task_test PRIVATE include)
  ament_target_dependencies(flight_task_test geometry_msgs uav_interfaces)
  add_test(NAME flight_task COMMAND flight_task_test)
endif()

ament_package()
