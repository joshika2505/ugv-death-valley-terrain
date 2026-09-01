#!/bin/bash
set -e

CONTAINER_NAME="sih_ugv_runner"
WS_DIR="/home/ubuntu/sih_ws"

function print_banner() {
    echo "=========================================================================="
    echo "  🏥 GPS-DENIED AUTONOMOUS UGV NAVIGATION: POINT A -> HOSPITAL POINT B 🏥"
    echo "  Architecture: ROS 2 Jazzy + Gazebo Harmonic + Visual SLAM + Nav2"
    echo "=========================================================================="
}

function stop_simulation() {
    echo "Stopping any running simulation and ROS 2 processes..."
    docker exec $CONTAINER_NAME pkill -9 -f "locality_|forest_|sih_|gemini_|dashboard_|ros2|gz|gazebo|rviz2|parameter_bridge|controller_server|planner_server|bt_navigator" 2>/dev/null || true
    pkill -9 -f "run_locality_mission|locality_simulation" 2>/dev/null || true
    echo "✓ Simulation stopped cleanly."
}

if [ "$1" == "stop" ]; then
    stop_simulation
    exit 0
fi

if [ "$1" == "status" ]; then
    echo "--- ROS 2 & System Status ---"
    docker exec $CONTAINER_NAME bash -c "source /opt/ros/jazzy/setup.bash && ros2 node list 2>/dev/null && ros2 topic list 2>/dev/null"
    exit 0
fi

print_banner
stop_simulation

echo "1. Checking Docker container..."
if [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Starting Docker container $CONTAINER_NAME..."
    docker start $CONTAINER_NAME
    sleep 2
fi

echo "2. Building ROS 2 Workspace inside container..."
docker exec $CONTAINER_NAME bash -c "
    source /opt/ros/jazzy/setup.bash
    cd $WS_DIR
    colcon build --symlink-install
"

echo "3. Launching GPS-Denied Autonomous Locality to Hospital Mission..."
echo "--------------------------------------------------------------------------"
echo "  Mission Target: Point A (0.0, 0.0) -> Hospital Point B (24.0, 8.0)"
echo "  Localization: GPS-DENIED (Pi-Cam Visual SLAM + IMU + Wheel Odometry)"
echo "  HERCULES Dashboard: http://localhost:8080"
echo "--------------------------------------------------------------------------"

docker exec -i $CONTAINER_NAME bash -c "
    source /opt/ros/jazzy/setup.bash
    source $WS_DIR/install/setup.bash
    export GZ_SIM_RESOURCE_PATH=$WS_DIR/install/forest_ugv_gazebo/share/forest_ugv_gazebo/worlds:$WS_DIR/install/forest_ugv_description/share
    ros2 launch forest_ugv_bringup locality_simulation.launch.py headless:=true rviz:=false
"
