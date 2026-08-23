cmake_minimum_required(VERSION 3.8)
project(mission_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(mission_control_node src/mission_control_node.cpp)
target_include_directories(mission_control_node PRIVATE include)
ament_target_dependencies(mission_control_node
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs)

install(TARGETS mission_control_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control test/test_control.cpp)
  target_include_directories(test_control PRIVATE include)
endif()

ament_package()
