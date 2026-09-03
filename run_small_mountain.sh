#!/bin/bash
# ==============================================================================
# Small Mountain 3D Environment Simulation Launcher for Gazebo
# ==============================================================================
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
cd "$DIR"

# Allow local X11 display connections
xhost +local:root > /dev/null 2>&1 || xhost + > /dev/null 2>&1 || true

CONTAINER_NAME="sih_ugv_runner"
IMAGE_NAME="sih_ugv:latest"
WORKSPACE_DIR="/home/ubuntu/sih_ws"

ensure_container() {
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        docker rm -f "$CONTAINER_NAME" > /dev/null 2>&1 || true
        echo "Starting simulation container..."
        docker run -it -d \
            --net=host \
            --name "$CONTAINER_NAME" \
            --privileged \
            --volume "$DIR":$WORKSPACE_DIR/src \
            --volume /dev:/dev \
            --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
            --volume /tmp:/tmp \
            --env="DISPLAY=$DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            --env="TERM=xterm-256color" \
            --user root \
            "$IMAGE_NAME" bash -c "mkdir -p $WORKSPACE_DIR/src && chown -R ubuntu:ubuntu $WORKSPACE_DIR && sleep infinity"
    fi
}

MODE="${1:-sim}"

echo "=========================================================="
echo "  ⛰️  Small Mountain Terrain Simulation in Gazebo"
echo "  Operational Mode: $MODE"
echo "=========================================================="

case "$MODE" in
    sim|gz|gazebo-sim)
        ensure_container
        echo "Launching Small Mountain Terrain in Gazebo Sim (Harmonic)..."
        docker exec -it \
            --env="DISPLAY=$DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            "$CONTAINER_NAME" bash -c "
                source /opt/ros/jazzy/setup.bash && \
                source $WORKSPACE_DIR/install/setup.bash && \
                export GZ_SIM_RESOURCE_PATH=$WORKSPACE_DIR/install/small_mountain_world/share/small_mountain_world:$WORKSPACE_DIR/install/small_mountain_world/share/small_mountain_world/models:\$GZ_SIM_RESOURCE_PATH && \
                ros2 launch small_mountain_world small_mountain.launch.py"
        ;;

    amr4|robot)
        ensure_container
        echo "Launching Small Mountain Terrain with AMR-4 Autonomous Rover..."
        docker exec -it \
            --env="DISPLAY=$DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            "$CONTAINER_NAME" bash -c "
                source /opt/ros/jazzy/setup.bash && \
                source $WORKSPACE_DIR/install/setup.bash && \
                export GZ_SIM_RESOURCE_PATH=$WORKSPACE_DIR/install/small_mountain_world/share/small_mountain_world:$WORKSPACE_DIR/install/small_mountain_world/share/small_mountain_world/models:\$GZ_SIM_RESOURCE_PATH && \
                ros2 launch small_mountain_world small_mountain_amr4.launch.py"
        ;;

    classic)
        echo "Launching Small Mountain Terrain in Gazebo Classic (v11) on host..."
        pkill -9 -f "gzserver|gzclient" 2>/dev/null || true
        export GAZEBO_MODEL_PATH="$DIR/src/small_mountain_world/models:$GAZEBO_MODEL_PATH"
        export GAZEBO_RESOURCE_PATH="$DIR/src/small_mountain_world:$GAZEBO_RESOURCE_PATH"
        export GAZEBO_MODEL_DATABASE_URI=""
        gazebo --verbose "$DIR/src/small_mountain_world/worlds/small_mountain.world"
        ;;

    stop)
        echo "Stopping simulation container and processes..."
        docker exec "$CONTAINER_NAME" pkill -9 -f "gz|gazebo|ros2" 2>/dev/null || true
        pkill -9 -f "gzserver|gzclient|gazebo" 2>/dev/null || true
        echo "Simulation stopped."
        ;;

    *)
        echo "Usage: ./run_small_mountain.sh [sim | amr4 | classic | stop]"
        ;;
esac
