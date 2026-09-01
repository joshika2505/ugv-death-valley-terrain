cmake_minimum_required(VERSION 3.8)
project(amr4_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/amr4_navigation
)

ament_package()
