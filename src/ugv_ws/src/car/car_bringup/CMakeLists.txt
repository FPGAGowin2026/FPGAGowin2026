cmake_minimum_required(VERSION 3.8)
project(car_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_parameters test/test_launch_parameters.py)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_bx72_bidirectional.py TIMEOUT 45)
  add_launch_test(test/test_multimonitor.py TIMEOUT 60)
  add_launch_test(test/test_navigation_mode_wait.py TIMEOUT 60)
  add_launch_test(test/test_fpga_mode_wait.py TIMEOUT 60)
  add_launch_test(test/test_navigation_lease_stop.py TIMEOUT 90)
  add_launch_test(test/test_jetson_nav_e2e.py TIMEOUT 60)
  add_launch_test(test/test_waypoint_ack_retry.py TIMEOUT 90)
  add_launch_test(test/test_global_nav_e2e.py TIMEOUT 60)
  add_launch_test(test/test_navigation_sync.py TIMEOUT 45)
  add_launch_test(test/test_ros_control.py TIMEOUT 60)
  add_launch_test(test/test_web_mission.py TIMEOUT 60)
endif()

ament_package()
