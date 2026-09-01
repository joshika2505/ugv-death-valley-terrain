cmake_minimum_required(VERSION 3.8)
project(ugv_belt_drive)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  launch
  worlds
  config
  rviz
  meshes
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ugv_autonomous_mission.py
  scripts/odom_to_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
