cmake_minimum_required(VERSION 3.8)
project(amr4_autonomy)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  amr4_autonomy/navigation_manager_node.py
  DESTINATION lib/amr4_autonomy
  RENAME navigation_manager_node
)

install(PROGRAMS
  amr4_autonomy/autonomous_goal_node.py
  DESTINATION lib/amr4_autonomy
  RENAME autonomous_goal_node
)

install(PROGRAMS
  amr4_autonomy/perception_display_node.py
  DESTINATION lib/amr4_autonomy
  RENAME perception_display_node
)

install(PROGRAMS
  amr4_autonomy/odom_tf_broadcaster.py
  DESTINATION lib/amr4_autonomy
  RENAME odom_tf_broadcaster
)

install(PROGRAMS
  amr4_autonomy/clock_filter.py
  DESTINATION lib/amr4_autonomy
  RENAME clock_filter
)

install(PROGRAMS
  amr4_autonomy/terrain_analyzer.py
  amr4_autonomy/terrain_path_planner.py
  amr4_autonomy/physics_controller.py
  DESTINATION lib/amr4_autonomy
)

install(
  DIRECTORY launch config web
  DESTINATION share/amr4_autonomy
)

ament_package()
