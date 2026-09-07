cmake_minimum_required(VERSION 3.8)
project(fpga_gateway)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(fpga_protocol src/protocol.cpp)
target_include_directories(fpga_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(fpga_gateway_node src/fpga_gateway_node.cpp)
target_link_libraries(fpga_gateway_node fpga_protocol)
ament_target_dependencies(fpga_gateway_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs)

add_executable(fake_fpga_node src/fake_fpga_node.cpp)
target_link_libraries(fake_fpga_node fpga_protocol)
ament_target_dependencies(fake_fpga_node rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS
  fpga_protocol
  fpga_gateway_node
  fake_fpga_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol fpga_protocol)
endif()

ament_package()
