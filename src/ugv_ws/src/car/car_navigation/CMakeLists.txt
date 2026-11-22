cmake_minimum_required(VERSION 3.8)
project(car_navigation)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(nlohmann_json REQUIRED)
set(ros_dependencies
  rclcpp rclcpp_action nav2_msgs nav_msgs geometry_msgs sensor_msgs std_msgs std_srvs tf2 tf2_ros)
foreach(dependency IN LISTS ros_dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_executable(robot_mission
  src/robot_mission.cpp
  src/fpga_reference.cpp
  src/grid_route.cpp
  src/navigation_world.cpp
  src/waypoint_follower.cpp)
target_compile_features(robot_mission PRIVATE cxx_std_17)
target_include_directories(robot_mission PRIVATE include)
ament_target_dependencies(robot_mission ${ros_dependencies})
target_link_libraries(robot_mission Threads::Threads nlohmann_json::nlohmann_json)
install(TARGETS robot_mission DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES mission_services.launch.py DESTINATION share/${PROJECT_NAME}/launch)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
