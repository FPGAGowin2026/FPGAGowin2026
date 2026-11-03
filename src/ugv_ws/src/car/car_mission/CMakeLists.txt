cmake_minimum_required(VERSION 3.18)
project(car_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(ugv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn)
find_package(Eigen3 REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h REQUIRED)
find_library(TENSORRT_NVINFER_LIBRARY NAMES nvinfer REQUIRED)

set(BYTETRACK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/bytetrack")
set(BYTETRACK_SOURCES
  ${BYTETRACK_ROOT}/src/BYTETracker.cpp
  ${BYTETRACK_ROOT}/src/STrack.cpp
  ${BYTETRACK_ROOT}/src/kalmanFilter.cpp
  ${BYTETRACK_ROOT}/src/lapjv.cpp
  ${BYTETRACK_ROOT}/src/utils.cpp)
add_library(car_bytetrack OBJECT ${BYTETRACK_SOURCES})
target_include_directories(car_bytetrack PUBLIC "${BYTETRACK_ROOT}/include" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(car_bytetrack PUBLIC Eigen3::Eigen ${OpenCV_LIBRARIES})

add_executable(vision_node src/vision_node.cpp)
target_include_directories(vision_node PRIVATE
  "${TENSORRT_INCLUDE_DIR}")
target_link_libraries(vision_node
  car_bytetrack
  "${TENSORRT_NVINFER_LIBRARY}"
  CUDA::cudart)
ament_target_dependencies(vision_node
  cv_bridge
  rclcpp
  sensor_msgs
  vision_msgs)

add_executable(mission_control_node src/mission_control_node.cpp)
target_include_directories(mission_control_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(mission_control_node ${OpenCV_LIBRARIES})
ament_target_dependencies(mission_control_node
  diagnostic_msgs
  cv_bridge
  sensor_msgs
  ugv_bridge
  geometry_msgs
  rclcpp
  std_msgs
  std_srvs
  vision_msgs)

install(TARGETS vision_node mission_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frame_processing_schedule test/test_frame_processing_schedule.cpp)

  ament_add_gtest(test_vision test/test_vision.cpp)
  if(TARGET test_vision)
    target_include_directories(test_vision PRIVATE
      "${TENSORRT_INCLUDE_DIR}")
    target_link_libraries(test_vision
      car_bytetrack
      "${TENSORRT_NVINFER_LIBRARY}"
      CUDA::cudart)
    ament_target_dependencies(test_vision
      cv_bridge
      rclcpp
      sensor_msgs
      vision_msgs)
  endif()

  ament_add_gtest(test_mission_control test/test_mission_control.cpp)
  if(TARGET test_mission_control)
    target_include_directories(test_mission_control PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(test_mission_control ${OpenCV_LIBRARIES})
    ament_target_dependencies(test_mission_control
      diagnostic_msgs
      cv_bridge
      sensor_msgs
      ugv_bridge
      geometry_msgs
      rclcpp
      std_msgs
      std_srvs
      vision_msgs)
    set_tests_properties(test_mission_control PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=94;ROS_LOCALHOST_ONLY=1")
  endif()
endif()

ament_package()
