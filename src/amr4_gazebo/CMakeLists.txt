cmake_minimum_required(VERSION 3.8)
project(amr4_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/amr4_gazebo
)

ament_package()
