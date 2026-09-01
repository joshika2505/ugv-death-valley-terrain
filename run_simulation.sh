#!/usr/bin/env bash
# ==============================================================================
# HERCULES AUTONOMOUS UGV - REAL-WORLD DIGITAL TWIN SIMULATION LAUNCHER
# Launches Gazebo 3D World + RViz2 Visualizer + Live Robot Camera GUI + Web Dashboard
# ==============================================================================

set -e

echo "======================================================================"
echo "  🚀 STARTING HERCULES REAL-WORLD DIGITAL-TWIN SIMULATION ON UBUNTU   "
echo "  Robot: MARBLE_HUSKY_SENSOR_CONFIG_1 (Clearpath 4WD Skid-Steer)       "
echo "  Environment: 500m x 500m Digital-Twin with Material Physics         "
echo "  Navigation: GPS-Denied Visual SLAM + Nav2 Dynamic Replanning         "
echo "======================================================================"

# 1. Allow X11 / Xwayland Display Access
xhost +local:root >/dev/null 2>&1 || xhost + >/dev/null 2>&1 || true

# 2. Kill any stale background simulation tasks
docker exec sih_ugv_runner pkill -9 -f "locality_|forest_|sih_|gemini_|dashboard_|ros2|gz|gazebo|rviz2|parameter_bridge|controller_server|planner_server|bt_navigator|camera_viewer_gui|point_ab|dynamic_obstacle|digital_twin" 2>/dev/null || true
sleep 1

# 3. Launch Core Autonomy, ROS 2 Bridges, Nav2, Visual SLAM, and Gazebo Server in Docker
echo "[1/4] Launching Core ROS 2 Autonomy & Simulation Backend..."
docker exec -d sih_ugv_runner bash -c "
    export DISPLAY=:0
    export GEMINI_API_KEY='YOUR_API_KEY_HERE'
    source /opt/ros/jazzy/setup.bash
    source /home/ubuntu/sih_ws/install/setup.bash
    export GZ_SIM_RESOURCE_PATH=/home/ubuntu/sih_ws/install/forest_ugv_gazebo/share/forest_ugv_gazebo/worlds:/home/ubuntu/sih_ws/install/forest_ugv_description/share:/home/ubuntu/sih_ws/install/forest_ugv_description/share/forest_ugv_description/models
    ros2 launch forest_ugv_bringup locality_simulation.launch.py headless:=false rviz:=true camera_gui:=false
"
sleep 4

# 4. Launch Gazebo 3D World Client Window on Host Desktop
echo "[2/4] Launching Gazebo 3D World Simulation Window..."
docker exec -d sih_ugv_runner bash -c "
    export DISPLAY=:0
    export GZ_SIM_RESOURCE_PATH=/home/ubuntu/sih_ws/install/forest_ugv_gazebo/share/forest_ugv_gazebo/worlds:/home/ubuntu/sih_ws/install/forest_ugv_description/share:/home/ubuntu/sih_ws/install/forest_ugv_description/share/forest_ugv_description/models
    source /opt/ros/jazzy/setup.bash
    gz sim -g
"
sleep 2

# 5. Launch Live Robot Camera GUI Viewport on Host Desktop
echo "[3/4] Launching Dedicated Robot Camera Eye Live Viewport..."
docker exec -d sih_ugv_runner bash -c "
    export DISPLAY=:0
    source /opt/ros/jazzy/setup.bash
    source /home/ubuntu/sih_ws/install/setup.bash
    ros2 run forest_perception camera_viewer_gui
"
sleep 1

echo "======================================================================"
echo "  ✓ ALL VISUAL WINDOWS ARE RUNNING ON YOUR UBUNTU DESKTOP SCREEN:     "
echo "    - 🪟 Gazebo Sim 3D (World View & MARBLE Husky UGV)                "
echo "    - 🪟 RViz2 (Autonomy Map, Point Clouds, TF, Nav2 Trajectory)      "
echo "    - 🪟 Robot Eye View (Live Camera Feed at 30 FPS)                  "
echo "    - 🌐 Web Dashboard at: http://localhost:8080                      "
echo "======================================================================"
