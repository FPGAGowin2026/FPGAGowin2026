cmake_minimum_required(VERSION 3.16)
project(swarm_cooperation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d OPTIONAL_COMPONENTS aruco objdetect)

ament_get_python_install_dir(python_dir)
pybind11_add_module(grid src/grid.cpp)
pybind11_add_module(_april src/april.cpp)
target_compile_features(grid PRIVATE cxx_std_17)
target_compile_features(_april PRIVATE cxx_std_17)
target_include_directories(_april PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(_april PRIVATE ${OpenCV_LIBS})
if(OpenCV_VERSION VERSION_GREATER_EQUAL "4.7" AND OpenCV_objdetect_FOUND)
  target_compile_definitions(_april PRIVATE SWARM_APRIL_ARUCO_MODERN)
elseif(OpenCV_aruco_FOUND)
  target_compile_definitions(_april PRIVATE SWARM_APRIL_ARUCO_LEGACY)
endif()

# Install the Python adapters and the native modules in the same package.
ament_python_install_package(${PROJECT_NAME})
install(TARGETS grid _april LIBRARY DESTINATION "${python_dir}/${PROJECT_NAME}")
# Humble copies renamed programs even with --symlink-install. Keep the ROS
# launcher small so that it always imports the installed (or linked) coordinator.
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/swarm_console.py"
  CONTENT "#!${Python3_EXECUTABLE}\nfrom swarm_cooperation.console import main\nmain()\n")
install(PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/swarm_console.py" DESTINATION lib/${PROJECT_NAME} RENAME swarm_console)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} PATTERN "__pycache__" EXCLUDE)
ament_package()
