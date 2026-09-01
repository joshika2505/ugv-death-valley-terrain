cmake_minimum_required(VERSION 3.8)
project(amr4_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch models urdf
  DESTINATION share/amr4_description
)

ament_package()
