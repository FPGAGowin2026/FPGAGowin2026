cmake_minimum_required(VERSION 3.8)
project(ugv_bridge)

# Sensor-rate JSON and ROS publication are part of the live control gateway.
# Keep symbols while avoiding an accidental unoptimized real-time data path.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bx72ImuRaw.msg" "msg/N10Packet.msg" "msg/EncoderCounts.msg" "msg/AuthorizedTwist.msg" "srv/MissionStart.srv" "srv/NavigationCommand.srv" "srv/GlobalNavigation.srv"
  DEPENDENCIES std_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)

add_library(bx72_protocol STATIC src/bx72_protocol.cpp)
target_include_directories(bx72_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(bx72_protocol PUBLIC nlohmann_json::nlohmann_json)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
add_executable(ugv_bridge_node src/ugv_bridge_node.cpp)
target_include_directories(ugv_bridge_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ugv_bridge_node
  bx72_protocol "${cpp_typesupport_target}" ${OpenCV_LIBS} Threads::Threads)
ament_target_dependencies(ugv_bridge_node diagnostic_msgs geometry_msgs nav_msgs rclcpp rmw sensor_msgs std_msgs std_srvs tf2_ros vision_msgs)

add_executable(ugv_debug src/ugv_debug.cpp)
target_include_directories(ugv_debug PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ugv_debug bx72_protocol Threads::Threads ${OpenCV_LIBS})

install(FILES include/ugv_bridge/bx72_protocol.hpp DESTINATION include/ugv_bridge)
install(FILES include/ugv_bridge/ros_navigation.hpp DESTINATION include/ugv_bridge)
install(FILES include/ugv_bridge/video.hpp DESTINATION include/ugv_bridge)
install(FILES include/ugv_bridge/route_overlay.hpp DESTINATION include/ugv_bridge)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(TARGETS ugv_bridge_node ugv_debug
  RUNTIME DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_authorized_twist test/test_authorized_twist.cpp)
  if(TARGET test_authorized_twist)
    target_include_directories(test_authorized_twist PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_authorized_twist "${cpp_typesupport_target}")
    ament_target_dependencies(test_authorized_twist geometry_msgs std_msgs)
  endif()
  ament_add_gtest(test_route_overlay test/test_route_overlay.cpp)
  if(TARGET test_route_overlay)
    target_include_directories(test_route_overlay PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(test_route_overlay nlohmann_json::nlohmann_json ${OpenCV_LIBS})
  endif()
  ament_add_gtest(test_navigation_protocol test/test_navigation_protocol.cpp)
  if(TARGET test_navigation_protocol)
    target_link_libraries(test_navigation_protocol bx72_protocol)
  endif()
  ament_add_gtest(test_video test/test_video.cpp)
  if(TARGET test_video)
    target_include_directories(test_video PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(test_video bx72_protocol ${OpenCV_LIBS})
  endif()
  ament_add_gtest(test_ros_navigation test/test_ros_navigation.cpp)
  if(TARGET test_ros_navigation)
    target_include_directories(test_ros_navigation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    ament_target_dependencies(test_ros_navigation geometry_msgs nav_msgs sensor_msgs)
  endif()
endif()

ament_package()
