cmake_minimum_required(VERSION 3.8)
project(death_valley_world)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch models meshes worlds textures
  DESTINATION share/death_valley_world
)

ament_package()
