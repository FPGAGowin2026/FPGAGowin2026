cmake_minimum_required(VERSION 3.8)
project(uav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# 仿真文件由工作区 sim/ 集中管理，仍安装到原 ROS 包。
install(DIRECTORY ../../sim/uav_bringup/launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
