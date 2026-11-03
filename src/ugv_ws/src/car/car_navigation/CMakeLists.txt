cmake_minimum_required(VERSION 3.8)
project(car_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ugv_bridge REQUIRED)
find_package(nlohmann_json REQUIRED)

# Jetson 局部图由航点执行器和外部调用方共用，不重复建图。
add_library(bx72_local_grid SHARED src/bx72_local_grid.c src/bx72_local_grid_trig.c)
set_target_properties(bx72_local_grid PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED YES)
target_include_directories(bx72_local_grid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS bx72_local_grid EXPORT export_bx72_local_grid
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_bx72_local_grid HAS_LIBRARY_TARGET)

# FPGA 与 Jetson 使用同一份局部 A*，适配层连接唯一的 Jetson 局部栅格。
get_filename_component(bx72_fpga_firmware
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../FPGA_ws/boards/BX72_138K_robot/src/firmware/lite" ABSOLUTE)
if(NOT EXISTS "${bx72_fpga_firmware}/local_path.c" OR NOT EXISTS "${bx72_fpga_firmware}/local_path.h")
  message(FATAL_ERROR "BX72 shared local_path source is missing: ${bx72_fpga_firmware}")
endif()
add_library(bx72_jetson_navigation STATIC src/nav_compat/navigation.c src/nav_compat/trig_adapter.c
  src/nav_compat/local_path_adapter.c)
set_target_properties(bx72_jetson_navigation PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED YES)
target_compile_definitions(bx72_jetson_navigation PRIVATE BX72_NAV_JETSON)
target_include_directories(bx72_jetson_navigation PRIVATE src/nav_compat include "${bx72_fpga_firmware}")
target_link_libraries(bx72_jetson_navigation bx72_local_grid)

add_executable(jetson_local_navigation src/jetson_local_navigation.cpp)
target_compile_features(jetson_local_navigation PRIVATE cxx_std_17)
target_include_directories(jetson_local_navigation PRIVATE src/nav_compat include)
target_link_libraries(jetson_local_navigation bx72_jetson_navigation bx72_local_grid nlohmann_json::nlohmann_json)
ament_target_dependencies(jetson_local_navigation rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs diagnostic_msgs ugv_bridge)
install(TARGETS jetson_local_navigation RUNTIME DESTINATION lib/${PROJECT_NAME})

# Symlink installs use the source file's mode and do not support PROGRAMS RENAME.
# Keep an executable copy in the build tree with its Python module alongside it.
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/global_map_source" "${CMAKE_CURRENT_BINARY_DIR}/global_map_runtime")
configure_file(scripts/global_map_node.py "${CMAKE_CURRENT_BINARY_DIR}/global_map_source/global_map_node.py" COPYONLY)
configure_file(scripts/global_map_core.py "${CMAKE_CURRENT_BINARY_DIR}/global_map_runtime/global_map_core.py" COPYONLY)
file(COPY "${CMAKE_CURRENT_BINARY_DIR}/global_map_source/global_map_node.py"
  DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/global_map_runtime"
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
install(PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/global_map_runtime/global_map_node.py" DESTINATION lib/${PROJECT_NAME})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/global_map_runtime/global_map_core.py" DESTINATION lib/${PROJECT_NAME})

# 全局路线计算和航点状态机在 Jetson 运行；网页仅提交和显示任务。
set(global_nav_runtime "${CMAKE_CURRENT_BINARY_DIR}/global_nav_runtime")
file(MAKE_DIRECTORY "${global_nav_runtime}/global_planner/public/js")
foreach(file global_navigation.mjs global_route.mjs global_route_worker.mjs
             fpga_grid_map.mjs jetson_global_planner.mjs route_runner.mjs
             public/js/grid_codec.js public/js/local_task_validation.js)
  configure_file("scripts/global_planner/${file}" "${global_nav_runtime}/global_planner/${file}" COPYONLY)
endforeach()
configure_file(scripts/jetson_global_navigation_node.py
               "${CMAKE_CURRENT_BINARY_DIR}/global_map_source/jetson_global_navigation_node.py" COPYONLY)
file(COPY "${CMAKE_CURRENT_BINARY_DIR}/global_map_source/jetson_global_navigation_node.py"
  DESTINATION "${global_nav_runtime}"
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
install(PROGRAMS "${global_nav_runtime}/jetson_global_navigation_node.py" DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY "${global_nav_runtime}/global_planner" DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs/ DESTINATION share/${PROJECT_NAME}/docs)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(test_global_map test/test_global_map.py)
  ament_add_pytest_test(test_global_safety_epoch test/test_global_safety_epoch.py)
  add_launch_test(test/test_jetson_local_node.py TIMEOUT 60)
  enable_testing()
  find_program(NODE_EXECUTABLE node REQUIRED)
  add_test(NAME test_jetson_global_planner
    COMMAND "${NODE_EXECUTABLE}" --test "${CMAKE_CURRENT_SOURCE_DIR}/test/test_jetson_global_planner.mjs")
  add_executable(test_bx72_local_grid test/test_bx72_local_grid.c)
  target_include_directories(test_bx72_local_grid PRIVATE src)
  target_link_libraries(test_bx72_local_grid bx72_local_grid)
  add_test(NAME test_bx72_local_grid COMMAND test_bx72_local_grid)
  add_executable(test_jetson_local_navigation test/test_jetson_local_navigation.c src/nav_compat/trig_adapter.c
    src/nav_compat/local_path_adapter.c)
  target_compile_definitions(test_jetson_local_navigation PRIVATE BX72_NAV_HOST_TEST BX72_NAV_TEST_TIMED)
  target_include_directories(test_jetson_local_navigation PRIVATE src/nav_compat include "${bx72_fpga_firmware}")
  target_link_libraries(test_jetson_local_navigation bx72_local_grid m)
  add_test(NAME test_jetson_local_navigation COMMAND test_jetson_local_navigation)
  add_executable(test_jetson_navigation_command test/test_jetson_navigation_command.cpp)
  target_compile_features(test_jetson_navigation_command PRIVATE cxx_std_17)
  target_compile_definitions(test_jetson_navigation_command PRIVATE BX72_NAV_COMMAND_TEST)
  target_include_directories(test_jetson_navigation_command PRIVATE src/nav_compat include)
  target_link_libraries(test_jetson_navigation_command bx72_jetson_navigation bx72_local_grid nlohmann_json::nlohmann_json)
  ament_target_dependencies(test_jetson_navigation_command rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs diagnostic_msgs ugv_bridge)
  add_test(NAME test_jetson_navigation_command COMMAND test_jetson_navigation_command)
endif()

ament_package()
