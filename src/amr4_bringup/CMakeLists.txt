cmake_minimum_required(VERSION 3.8)
project(amr4_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz config
  DESTINATION share/amr4_bringup
)

ament_package()
