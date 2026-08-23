cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(sim_laser_node src/sim_laser_node.cpp)
target_include_directories(sim_laser_node PRIVATE include)
ament_target_dependencies(sim_laser_node nav_msgs rclcpp sensor_msgs)

install(TARGETS sim_laser_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_include_directories(test_geometry PRIVATE include)
endif()

ament_package()
