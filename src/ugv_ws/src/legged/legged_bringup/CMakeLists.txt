cmake_minimum_required(VERSION 3.8)
project(legged_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
