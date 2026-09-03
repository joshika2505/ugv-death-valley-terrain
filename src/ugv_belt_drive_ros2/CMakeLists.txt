cmake_minimum_required(VERSION 3.8)
project(ugv_belt_drive_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf meshes config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
