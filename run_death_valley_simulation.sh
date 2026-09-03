#!/bin/bash
# ==============================================================================
# Master Autonomous AMR-4 Simulation Launcher in Death Valley Gazebo
# ==============================================================================
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
cd "$DIR"

# Allow local X11 display connections
xhost +local: > /dev/null 2>&1 || true

CONTAINER_NAME="sih_ugv_runner"
IMAGE_NAME="sih_ugv:latest"
WORKSPACE_DIR="/home/ubuntu/sih_ws"

# Check if container is running, otherwise start it
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    docker rm -f "$CONTAINER_NAME" > /dev/null 2>&1 || true

    echo "=========================================================="
    echo " Starting AMR-4 Death Valley Docker Simulation Container..."
    echo "=========================================================="
    docker run -it -d         --net=host         --name "$CONTAINER_NAME"         --privileged         --volume "$DIR":$WORKSPACE_DIR/src         --volume /dev:/dev         --volume /tmp/.X11-unix:/tmp/.X11-unix:rw         --volume /tmp:/tmp         --env="DISPLAY=$DISPLAY"         --env="QT_X11_NO_MITSHM=1"         --env="TERM=xterm-256color"         --user root         "$IMAGE_NAME" bash -c "mkdir -p $WORKSPACE_DIR/src && chown -R ubuntu:ubuntu $WORKSPACE_DIR && sleep infinity"
fi

MODE="${1:-full}"

echo "=========================================================="
echo " AMR-4 Autonomous Navigation in Death Valley"
echo " Operational Mode: $MODE"
echo "=========================================================="

case "$MODE" in
    build)
        echo "Building ROS 2 packages..."
        docker exec -it "$CONTAINER_NAME" bash -c "            source /opt/ros/jazzy/setup.bash &&             cd $WORKSPACE_DIR &&             colcon build --symlink-install --packages-select death_valley_world amr4_description amr4_gazebo amr4_navigation amr4_autonomy amr4_bringup &&             echo 'Build complete!'"
        ;;

    full|autonomous|auto)
        START_X="${2:-0.0}"
        START_Y="${3:-0.0}"
        GOAL_X="${4:-20.0}"
        GOAL_Y="${5:-20.0}"
        echo "=========================================================="
        echo " Point A (Start): ($START_X, $START_Y)"
        echo " Point B (Stop):  ($GOAL_X, $GOAL_Y)"
        echo "=========================================================="
        echo "Launching Full Autonomous Simulation (Gazebo + AMR-4 + SLAM + Nav2 + RViz2 + Autonomy)..."
        docker exec \
            --env="DISPLAY=$DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            --env="ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" \
            --env="ROS_DOMAIN_ID=0" \
            "$CONTAINER_NAME" bash -c "
                pkill -9 -f ros2; pkill -9 -f gz; pkill -9 -f rviz; pkill -9 -f parameter_bridge; pkill -9 -f python3 || true; sleep 1;
                source /opt/ros/jazzy/setup.bash && \
                cd $WORKSPACE_DIR && \
                colcon build --symlink-install --packages-select death_valley_world amr4_description amr4_gazebo amr4_navigation amr4_autonomy amr4_bringup sih_bot && \
                source $WORKSPACE_DIR/install/setup.bash && \
                ros2 launch amr4_bringup death_valley_autonomous.launch.py start_x:=$START_X start_y:=$START_Y goal_x:=$GOAL_X goal_y:=$GOAL_Y"
        ;;

    sim|gazebo)
        echo "Launching Gazebo Death Valley Terrain & AMR-4 only..."
        docker exec \
            --env="DISPLAY=$DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            "$CONTAINER_NAME" bash -c "
                source /opt/ros/jazzy/setup.bash && \
                cd $WORKSPACE_DIR && \
                colcon build --symlink-install --packages-select death_valley_world amr4_description amr4_gazebo amr4_navigation amr4_autonomy amr4_bringup && \
                source $WORKSPACE_DIR/install/setup.bash && \
                ros2 launch amr4_gazebo sim.launch.py"
        ;;

    test)
        echo "Running Automated Autonomy Validation Test Suite..."
        docker exec \
            --env="DISPLAY=$DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            "$CONTAINER_NAME" bash -c "
                source /opt/ros/jazzy/setup.bash && \
                cd $WORKSPACE_DIR && \
                colcon build --symlink-install --packages-select death_valley_world amr4_description amr4_gazebo amr4_navigation amr4_autonomy amr4_bringup && \
                source $WORKSPACE_DIR/install/setup.bash && \
                python3 $WORKSPACE_DIR/src/test_autonomous_navigation.py"
        ;;

    stop)
        echo "Stopping simulation container..."
        docker rm -f "$CONTAINER_NAME" > /dev/null 2>&1 || true
        echo "Simulation stopped."
        ;;

    *)
        echo "Usage: ./run_death_valley_simulation.sh [full | sim | build | test | stop] [start_x start_y goal_x goal_y]"
        ;;
esac
