cmake_minimum_required(VERSION 3.8)
project(small_mountain_world)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch models meshes worlds textures
  DESTINATION share/small_mountain_world
)

ament_package()
