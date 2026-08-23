cmake_minimum_required(VERSION 3.8)
project(vision_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(vision_core src/detector.cpp)
target_include_directories(vision_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(red_target_node src/red_target_node.cpp)
target_link_libraries(red_target_node vision_core)
ament_target_dependencies(red_target_node geometry_msgs rclcpp sensor_msgs)

add_executable(synthetic_camera_node src/synthetic_camera_node.cpp)
target_link_libraries(synthetic_camera_node vision_core)
ament_target_dependencies(synthetic_camera_node nav_msgs rclcpp sensor_msgs)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(TARGETS
  vision_core
  red_target_node
  synthetic_camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_detector test/test_detector.cpp)
  target_link_libraries(test_detector vision_core)
endif()

ament_package()
