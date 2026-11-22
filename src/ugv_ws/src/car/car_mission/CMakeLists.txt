cmake_minimum_required(VERSION 3.18)
project(car_mission)

find_package(ament_cmake REQUIRED)

option(BUILD_TENSORRT_VISION "Build the Jetson TensorRT detector" ON)
if(BUILD_TENSORRT_VISION)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(vision_msgs REQUIRED)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn)
  find_package(Eigen3 REQUIRED)
  find_package(CUDAToolkit REQUIRED)
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h REQUIRED)
  find_library(TENSORRT_NVINFER_LIBRARY NAMES nvinfer REQUIRED)

  set(BYTETRACK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/bytetrack")
  add_executable(vision_node
    src/vision_node.cpp
    ${BYTETRACK_ROOT}/src/BYTETracker.cpp
    ${BYTETRACK_ROOT}/src/STrack.cpp
    ${BYTETRACK_ROOT}/src/kalmanFilter.cpp
    ${BYTETRACK_ROOT}/src/lapjv.cpp
    ${BYTETRACK_ROOT}/src/utils.cpp)
  target_compile_features(vision_node PRIVATE cxx_std_17)
  target_compile_options(vision_node PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(vision_node PRIVATE
    include "${BYTETRACK_ROOT}/include" "${TENSORRT_INCLUDE_DIR}" ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(vision_node Eigen3::Eigen ${OpenCV_LIBRARIES} "${TENSORRT_NVINFER_LIBRARY}" CUDA::cudart)
  ament_target_dependencies(vision_node rclcpp sensor_msgs vision_msgs)
  install(TARGETS vision_node RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(PROGRAMS scripts/vision_mission.py DESTINATION lib/${PROJECT_NAME})
ament_package()
