#!/bin/bash
# ==============================================================================
# Master Simulation & Autonomous Navigation Launcher for SIH Outdoor UGV
# Vision-Based Autonomous Navigation for GPS-Denied Outdoor Environment
# ==============================================================================
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
cd "$DIR"

# Allow X11 display from container
xhost +local: > /dev/null 2>&1 || true

CONTAINER_NAME="sih_ugv_runner"
IMAGE_NAME="sih_ugv:latest"
WORKSPACE_DIR="/home/ubuntu/sih_ws"

# Check if container is running, otherwise start it
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    # Remove existing stopped container if present
    docker rm -f "$CONTAINER_NAME" > /dev/null 2>&1 || true

    echo "=========================================================="
    echo " Starting SIH UGV Docker Simulation Container..."
    echo "=========================================================="
    docker run -it -d \
        --net=host \
        --name "$CONTAINER_NAME" \
        --privileged \
        --volume "$DIR":$WORKSPACE_DIR/src \
        --volume /dev:/dev \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="TERM=xterm-256color" \
        --user root \
        "$IMAGE_NAME" bash -c "mkdir -p $WORKSPACE_DIR/src && chown -R ubuntu:ubuntu $WORKSPACE_DIR && sleep infinity"
fi

MODE="${1:-full}"

echo "=========================================================="
echo " SIH Vision-Based Autonomous Navigation Stack"
echo " Operational Mode: $MODE"
echo "=========================================================="

case "$MODE" in
    build)
        echo "Building SIH ROS 2 packages with colcon..."
        docker exec -it "$CONTAINER_NAME" bash -c "\
            source /opt/ros/jazzy/setup.bash && \
            cd $WORKSPACE_DIR && \
            colcon build --symlink-install && \
            echo 'Build complete!'"
        ;;

    full|navigation|auto)
        echo "Launching Full Autonomous Simulation (Gazebo + RViz2 + AI Perception + Navigator)..."
        docker exec -it \
            --env="DISPLAY=$DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            "$CONTAINER_NAME" bash -c "\
                source /opt/ros/jazzy/setup.bash && \
                cd $WORKSPACE_DIR && \
                colcon build --symlink-install && \
                source $WORKSPACE_DIR/install/setup.bash && \
                ros2 launch sih_ugv_navigation navigation.launch.py"
        ;;

    sim|gazebo)
        echo "Launching Gazebo Outdoor Simulation World with UGV..."
        docker exec -it \
            --env="DISPLAY=$DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            "$CONTAINER_NAME" bash -c "\
                source /opt/ros/jazzy/setup.bash && \
                cd $WORKSPACE_DIR && \
                colcon build --symlink-install && \
                source $WORKSPACE_DIR/install/setup.bash && \
                ros2 launch sih_ugv_gazebo sim_outdoor.launch.py rviz:=true"
        ;;

    perception|vision)
        echo "Launching Gazebo World + AI Path Segmentation..."
        docker exec -it \
            --env="DISPLAY=$DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            "$CONTAINER_NAME" bash -c "\
                source /opt/ros/jazzy/setup.bash && \
                cd $WORKSPACE_DIR && \
                colcon build --symlink-install && \
                source $WORKSPACE_DIR/install/setup.bash && \
                ros2 launch sih_ugv_navigation navigation.launch.py auto_nav:=false"
        ;;

    teleop)
        echo "Launching Interactive Keyboard Teleoperation..."
        docker exec -it \
            "$CONTAINER_NAME" bash -c "\
                source /opt/ros/jazzy/setup.bash && \
                ros2 run teleop_twist_keyboard teleop_twist_keyboard"
        ;;

    test)
        echo "Running Automated SIH Verification Test Suite..."
        docker exec -it \
            "$CONTAINER_NAME" bash -c "\
                source /opt/ros/jazzy/setup.bash && \
                cd $WORKSPACE_DIR && \
                colcon build --symlink-install && \
                source $WORKSPACE_DIR/install/setup.bash && \
                python3 $WORKSPACE_DIR/src/test_perception_ai.py && \
                echo 'Verification tests passed successfully!'"
        ;;

    stop)
        echo "Stopping SIH container..."
        docker rm -f "$CONTAINER_NAME" > /dev/null 2>&1 || true
        echo "Container stopped."
        ;;

    *)
        echo "Usage: ./run_sih_simulation.sh [full | sim | perception | teleop | test | build | stop]"
        echo "  full        : Launch complete Gazebo + RViz2 + AI Perception + Autonomous Navigation (default)"
        echo "  sim         : Launch Gazebo outdoor terrain & UGV only"
        echo "  perception  : Launch Gazebo + AI Path Segmentation (without auto navigation)"
        echo "  teleop      : Control UGV with keyboard teleoperation"
        echo "  test        : Run automated test suite verifying AI inference and data pipelines"
        echo "  build       : Build ROS 2 workspace"
        echo "  stop        : Stop simulation container"
        ;;
esac
