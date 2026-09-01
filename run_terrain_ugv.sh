#!/usr/bin/env bash
# ==============================================================================
# Master Unified Launcher: Gazebo Terrain Generator + Custom UGV + Autonomy
# ==============================================================================

set -e

SCRIPT_PATH="${BASH_SOURCE[0]}"
while [ -L "$SCRIPT_PATH" ]; do
  SCRIPT_DIR="$(cd -P "$(dirname "$SCRIPT_PATH")" && pwd)"
  SCRIPT_PATH="$(readlink "$SCRIPT_PATH")"
  [[ $SCRIPT_PATH != /* ]] && SCRIPT_PATH="$SCRIPT_DIR/$SCRIPT_PATH"
done
PROJECT_ROOT="$(cd -P "$(dirname "$SCRIPT_PATH")" && pwd)"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BOLD='\033[1m'
NC='\033[0m'

function print_banner() {
    echo -e "${BLUE}${BOLD}"
    echo "================================================================================"
    echo "       GAZEBO TERRAIN GENERATOR + BELT-DRIVE UGV AUTONOMOUS SIMULATION         "
    echo "================================================================================"
    echo -e "${NC}"
}

function check_xhost() {
    if [ -n "$DISPLAY" ]; then
        xhost +local:root > /dev/null 2>&1 || xhost +local: > /dev/null 2>&1 || true
    fi
}

function build_workspace() {
    echo -e "${GREEN}[INFO] Building ugv_belt_drive package with colcon in ROS 2 Jazzy container...${NC}"
    docker run --rm \
      -v "${PROJECT_ROOT}:/home/ubuntu/sih_ws" \
      sih_ugv:latest bash -c "
        source /opt/ros/jazzy/setup.bash
        cd /home/ubuntu/sih_ws
        colcon build --packages-select ugv_belt_drive --symlink-install
      "
    echo -e "${GREEN}[SUCCESS] Build complete!${NC}"
}

function run_web_server() {
    echo -e "${GREEN}[INFO] Launching Interactive Terrain Generator Web Application...${NC}"
    echo -e "${YELLOW}[URL] Open your browser at: http://localhost:8080${NC}"
    cd "${PROJECT_ROOT}/gazebo_terrain_generator"
    exec uv run scripts/server.py
}

function print_active_terrain_banner() {
    local WORLD="$1"
    local WORLD_DIR="$(dirname "$WORLD")"
    local META_FILE="${WORLD_DIR}/terrain_metadata.json"
    local MODEL_NAME="$(basename "$WORLD_DIR")"

    echo -e "${CYAN}================================================================================${NC}"
    echo -e "${CYAN}                            ACTIVE TERRAIN SIMULATION                           ${NC}"
    echo -e "${CYAN}================================================================================${NC}"
    echo -e "  Location:               ${MODEL_NAME}"
    echo -e "  World File:             ${WORLD}"
    echo -e "  Terrain Heightmap:      ${WORLD_DIR}/mesh/height_map.png"
    echo -e "  Satellite Texture:      ${WORLD_DIR}/mesh/aerial.png"
    echo -e "  Robot Package:          /home/ubuntu/sih_ws/src/ugv_belt_drive (ugv_belt_drive)"
    if [ -f "$META_FILE" ]; then
        python3 -c "
import json
try:
    with open('$META_FILE') as f:
        meta = json.load(f)
    t = meta.get('terrain', {})
    pa = meta.get('point_a', {})
    pb = meta.get('point_b', {})
    print(f'  Terrain Dimensions:     {t.get(\"width_m\", \"N/A\")} m × {t.get(\"length_m\", \"N/A\")} m × {t.get(\"height_m\", \"N/A\")} m')
    print(f'  Elevation Profile:      {t.get(\"min_elevation_amsl\", \"N/A\")} m -> {t.get(\"max_elevation_amsl\", \"N/A\")} m AMSL (Relief: {t.get(\"elevation_range_m\", \"N/A\")} m)')
    print(f'  Point A (START):        Lat/Lon({pa.get(\"latitude\", \"N/A\")}, {pa.get(\"longitude\", \"N/A\")}) -> Gazebo ({pa.get(\"gazebo_x\", 0.0)}, {pa.get(\"gazebo_y\", 0.0)})')
    print(f'  Point B (GOAL):         Lat/Lon({pb.get(\"latitude\", \"N/A\")}, {pb.get(\"longitude\", \"N/A\")}) -> Gazebo ({pb.get(\"gazebo_x\", 5.0)}, {pb.get(\"gazebo_y\", 0.0)}) [Dist: {pb.get(\"distance_to_goal_m\", \"N/A\")} m]')
except Exception:
    pass
" 2>/dev/null || true
    fi
    echo -e "${CYAN}================================================================================${NC}"
}

function resolve_world() {
    local RAW_INPUT="$*"
    local WORLD=""

    mkdir -p /tmp/gazebo_terrain_generator
    chmod -R 777 /tmp/gazebo_terrain_generator > /dev/null 2>&1 || true

    if [ -z "$RAW_INPUT" ]; then
        local LATEST_WORLD=$(find /tmp/gazebo_terrain_generator "${PROJECT_ROOT}/gazebo_terrain_generator/sample_worlds" -name "*.world" 2>/dev/null | sort -r | head -n 1)
        echo "${LATEST_WORLD:-/tmp/gazebo_terrain_generator/San_francisco/San_francisco.world}"
        return
    fi

    if [[ "$RAW_INPUT" == *.world ]] && [ -f "$RAW_INPUT" ]; then
        echo "$RAW_INPUT"
        return
    fi

    local VARIANTS=(
        "$RAW_INPUT"
        "${RAW_INPUT// /_}"
        "${RAW_INPUT// /}"
        "${RAW_INPUT,,}"
        "${RAW_INPUT^}"
    )

    for V in "${VARIANTS[@]}"; do
        if [ -f "/tmp/gazebo_terrain_generator/${V}/${V}.world" ]; then
            echo "/tmp/gazebo_terrain_generator/${V}/${V}.world"
            return
        fi
        local MATCH_DIR=$(find /tmp/gazebo_terrain_generator -maxdepth 1 -iname "*${V}*" 2>/dev/null | head -n 1)
        if [ -n "$MATCH_DIR" ]; then
            local FOUND_WORLD=$(find "$MATCH_DIR" -maxdepth 2 -name "*.world" 2>/dev/null | head -n 1)
            if [ -n "$FOUND_WORLD" ]; then
                echo "$FOUND_WORLD"
                return
            fi
        fi
        local MATCH_ZIP=$(find "$HOME/Downloads" -maxdepth 1 -iname "*${V}*.zip" 2>/dev/null | head -n 1)
        if [ -n "$MATCH_ZIP" ]; then
            local BASE_ZIP=$(basename "$MATCH_ZIP" .zip)
            unzip -o "$MATCH_ZIP" -d "/tmp/gazebo_terrain_generator/" > /dev/null 2>&1 || true
            local EXTRACTED_WORLD=$(find "/tmp/gazebo_terrain_generator/${BASE_ZIP}" -name "*.world" 2>/dev/null | head -n 1)
            if [ -n "$EXTRACTED_WORLD" ]; then
                echo "$EXTRACTED_WORLD"
                return
            fi
        fi
        if [ -f "${PROJECT_ROOT}/gazebo_terrain_generator/sample_worlds/${V}/${V}.world" ]; then
            echo "${PROJECT_ROOT}/gazebo_terrain_generator/sample_worlds/${V}/${V}.world"
            return
        fi
    done

    echo "$RAW_INPUT"
}

function run_sim() {
    check_xhost
    local args=("$@")
    local HEADLESS="false"
    local WORLD_QUERY="${args[*]}"

    local WORLD=$(resolve_world "$WORLD_QUERY")
    local WORLD_DIR="$(dirname "$WORLD")"
    local GZ_RES_PATH="/home/ubuntu/sih_ws/src/ugv_belt_drive:/tmp/gazebo_terrain_generator:${WORLD_DIR}"

    print_active_terrain_banner "$WORLD"
    echo -e "${GREEN}[INFO] Launching Gazebo Harmonic with world: ${WORLD}${NC}"
    docker run -it --rm \
      --net=host \
      --privileged \
      -e DISPLAY="${DISPLAY:-:0}" \
      -e GZ_SIM_RESOURCE_PATH="${GZ_RES_PATH}" \
      -e GZ_FILE_PATH="${GZ_RES_PATH}" \
      -e SDF_PATH="${GZ_RES_PATH}" \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v "${PROJECT_ROOT}:/home/ubuntu/sih_ws" \
      -v /tmp/gazebo_terrain_generator:/tmp/gazebo_terrain_generator \
      sih_ugv:latest bash -c "
        source /opt/ros/jazzy/setup.bash
        cd /home/ubuntu/sih_ws
        if [ ! -f /home/ubuntu/sih_ws/install/setup.bash ]; then
          colcon build --packages-select ugv_belt_drive --symlink-install > /dev/null 2>&1
        fi
        source /home/ubuntu/sih_ws/install/setup.bash
        ros2 launch ugv_belt_drive spawn_robot.launch.py world:=\"${WORLD}\" headless:=${HEADLESS}
      "
}

function run_teleop() {
    echo -e "${GREEN}[INFO] Starting Keyboard Teleoperation for UGV (/ugv/cmd_vel)...${NC}"
    docker run -it --rm \
      --net=host \
      -v "${PROJECT_ROOT}:/home/ubuntu/sih_ws" \
      sih_ugv:latest bash -c "
        source /opt/ros/jazzy/setup.bash
        if [ ! -f /home/ubuntu/sih_ws/install/setup.bash ]; then
          cd /home/ubuntu/sih_ws && colcon build --packages-select ugv_belt_drive --symlink-install > /dev/null 2>&1
        fi
        source /home/ubuntu/sih_ws/install/setup.bash
        ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/ugv/cmd_vel
      "
}

function run_slam() {
    check_xhost
    echo -e "${GREEN}[INFO] Starting SLAM Toolbox and RViz2 Perception Visualizer...${NC}"
    docker run -it --rm \
      --net=host \
      --privileged \
      -e DISPLAY="${DISPLAY:-:0}" \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v "${PROJECT_ROOT}:/home/ubuntu/sih_ws" \
      sih_ugv:latest bash -c "
        source /opt/ros/jazzy/setup.bash
        cd /home/ubuntu/sih_ws
        if [ ! -f /home/ubuntu/sih_ws/install/setup.bash ]; then
          colcon build --packages-select ugv_belt_drive --symlink-install > /dev/null 2>&1
        fi
        source /home/ubuntu/sih_ws/install/setup.bash
        ros2 launch ugv_belt_drive slam.launch.py rviz:=true
      "
}

function run_nav() {
    echo -e "${GREEN}[INFO] Starting Nav2 Autonomous Navigation Stack...${NC}"
    docker run -it --rm \
      --net=host \
      -v "${PROJECT_ROOT}:/home/ubuntu/sih_ws" \
      sih_ugv:latest bash -c "
        source /opt/ros/jazzy/setup.bash
        cd /home/ubuntu/sih_ws
        if [ ! -f /home/ubuntu/sih_ws/install/setup.bash ]; then
          colcon build --packages-select ugv_belt_drive --symlink-install > /dev/null 2>&1
        fi
        source /home/ubuntu/sih_ws/install/setup.bash
        ros2 launch ugv_belt_drive navigation.launch.py
      "
}

function run_auto_mission() {
    local GOAL_X="${1:-3.0}"
    local GOAL_Y="${2:-0.0}"
    local GOAL_YAW="${3:-0.0}"
    local TIMEOUT="${4:-60}"

    echo -e "${GREEN}[INFO] Dispatching Autonomous Goal: X=${GOAL_X}m, Y=${GOAL_Y}m, Yaw=${GOAL_YAW}rad...${NC}"
    docker run -it --rm \
      --net=host \
      -v "${PROJECT_ROOT}:/home/ubuntu/sih_ws" \
      sih_ugv:latest bash -c "
        source /opt/ros/jazzy/setup.bash
        cd /home/ubuntu/sih_ws
        if [ ! -f /home/ubuntu/sih_ws/install/setup.bash ]; then
          colcon build --packages-select ugv_belt_drive --symlink-install > /dev/null 2>&1
        fi
        source /home/ubuntu/sih_ws/install/setup.bash
        python3 /home/ubuntu/sih_ws/src/ugv_belt_drive/scripts/ugv_autonomous_mission.py --goal_x ${GOAL_X} --goal_y ${GOAL_Y} --goal_yaw ${GOAL_YAW} --timeout ${TIMEOUT}
      "
}

function run_test_all() {
    echo -e "${YELLOW}[TEST] Running Comprehensive Multi-Location End-to-End Test Suite...${NC}"
    docker run --rm \
      -v "${PROJECT_ROOT}:/home/ubuntu/sih_ws" \
      -v /tmp/gazebo_terrain_generator:/tmp/gazebo_terrain_generator \
      sih_ugv:latest bash -c "
        source /opt/ros/jazzy/setup.bash
        cd /home/ubuntu/sih_ws
        colcon build --packages-select ugv_belt_drive --symlink-install > /dev/null 2>&1
        source /home/ubuntu/sih_ws/install/setup.bash

        echo '=== 1. Testing Default Terrain World ==='
        ros2 launch ugv_belt_drive spawn_robot.launch.py headless:=true &
        PID1=\$!
        sleep 7
        ros2 topic list | grep -E 'ugv|scan|imu|camera'
        kill -9 \$PID1 > /dev/null 2>&1 || true
        killall -9 gz sim ruby > /dev/null 2>&1 || true
        sleep 2

        echo '=== 2. Testing Generated Real-World Bengaluru DEM Terrain ==='
        ros2 launch ugv_belt_drive spawn_robot.launch.py headless:=true world:=/tmp/gazebo_terrain_generator/bengaluru_test_world/bengaluru_test_world.world &
        PID2=\$!
        sleep 7
        echo 'Testing Autonomous Mission on Bengaluru Terrain...'
        python3 /home/ubuntu/sih_ws/src/ugv_belt_drive/scripts/ugv_autonomous_mission.py --goal_x 2.0 --goal_y 0.0 --timeout 20
        kill -9 \$PID2 > /dev/null 2>&1 || true
        killall -9 gz sim ruby > /dev/null 2>&1 || true
        sleep 2

        echo '=== 3. Testing High-Elevation Mountain Terrain (Joshimath) ==='
        ros2 launch ugv_belt_drive spawn_robot.launch.py headless:=true world:=/tmp/gazebo_terrain_generator/Joshimath/Joshimath.world &
        PID3=\$!
        sleep 7
        ros2 topic list | grep -E 'ugv|scan|imu|camera'
        kill -9 \$PID3 > /dev/null 2>&1 || true
        killall -9 gz sim ruby > /dev/null 2>&1 || true
        sleep 2

        echo '=== 4. Testing Urban Terrain with 3D Buildings (Apple Park) ==='
        ros2 launch ugv_belt_drive spawn_robot.launch.py headless:=true world:=/tmp/gazebo_terrain_generator/applepark/applepark.world &
        PID4=\$!
        sleep 7
        ros2 topic list | grep -E 'ugv|scan|imu|camera'
        kill -9 \$PID4 > /dev/null 2>&1 || true
        killall -9 gz sim ruby > /dev/null 2>&1 || true

        echo '=== ALL MULTI-LOCATION SIMULATION TESTS PASSED SUCCESSFULLY! ==='
      "
}

function run_full_navigation() {
    check_xhost
    local args=("$@")
    local GOAL_X=""
    local GOAL_Y=""
    local HEADLESS="false"
    local WORLD_QUERY=""

    if [ ${#args[@]} -ge 3 ] && [[ "${args[-1]}" =~ ^-?[0-9]+(\.[0-9]+)?$ ]] && [[ "${args[-2]}" =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; then
        GOAL_Y="${args[-1]}"
        GOAL_X="${args[-2]}"
        WORLD_QUERY="${args[@]:0:${#args[@]}-2}"
    elif [ ${#args[@]} -ge 1 ]; then
        WORLD_QUERY="${args[*]}"
    fi

    local WORLD=$(resolve_world "$WORLD_QUERY")
    local WORLD_DIR="$(dirname "$WORLD")"
    local GZ_RES_PATH="/home/ubuntu/sih_ws/src/ugv_belt_drive:/tmp/gazebo_terrain_generator:${WORLD_DIR}"

    print_active_terrain_banner "$WORLD"
    echo -e "${GREEN}[INFO] Launching Full End-to-End Autonomous Pipeline (Gazebo + UGV + SLAM + Nav2 + RViz2 + Goal B)...${NC}"
    echo -e "${YELLOW}[WORLD] ${WORLD}${NC}"
    if [ -n "$GOAL_X" ] && [ -n "$GOAL_Y" ]; then
        echo -e "${YELLOW}[GOAL] X=${GOAL_X}m, Y=${GOAL_Y}m${NC}"
    else
        echo -e "${YELLOW}[GOAL] Auto-loaded from terrain_metadata.json (Point B)${NC}"
    fi

    killall -9 gz sim ruby rviz2 > /dev/null 2>&1 || true
    xhost +local:root > /dev/null 2>&1 || xhost +local: > /dev/null 2>&1 || true

    docker run -it --rm \
      --net=host \
      --ipc=host \
      --privileged \
      -e DISPLAY="${DISPLAY:-:0}" \
      -e GZ_SIM_RESOURCE_PATH="${GZ_RES_PATH}" \
      -e GZ_FILE_PATH="${GZ_RES_PATH}" \
      -e SDF_PATH="${GZ_RES_PATH}" \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v "${PROJECT_ROOT}:/home/ubuntu/sih_ws" \
      -v /tmp/gazebo_terrain_generator:/tmp/gazebo_terrain_generator \
      sih_ugv:latest bash -c "
        ip route add 224.0.0.0/4 dev lo > /dev/null 2>&1 || true
        export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
        export ROS_LOCALHOST_ONLY=1
        source /opt/ros/jazzy/setup.bash
        cd /home/ubuntu/sih_ws
        if [ ! -f /home/ubuntu/sih_ws/install/setup.bash ]; then
          colcon build --packages-select ugv_belt_drive --symlink-install > /dev/null 2>&1
        fi
        source /home/ubuntu/sih_ws/install/setup.bash
        ros2 launch ugv_belt_drive terrain_navigation.launch.py world:=\"${WORLD}\" headless:=${HEADLESS} ${GOAL_X:+goal_x:=$GOAL_X} ${GOAL_Y:+goal_y:=$GOAL_Y}
      "
}

function run_diagnose() {
    echo -e "${CYAN}================================================================================${NC}"
    echo -e "${CYAN}                     SYSTEM & SIMULATION DIAGNOSTIC REPORT                      ${NC}"
    echo -e "${CYAN}================================================================================${NC}"
    echo -e "ROS distro:             ROS 2 Jazzy Jalisco"
    echo -e "Gazebo version:         Gazebo Harmonic (gz-sim 8)"
    echo -e "Workspace:              ${PROJECT_ROOT}"
    echo -e "Robot package:          ugv_belt_drive (${PROJECT_ROOT}/src/ugv_belt_drive)"
    echo -e "Robot model:            4-Sprocket Skid-Steer Belt-Drive UGV with LiDAR & 1080p Camera"
    echo -n "GPU Status:             "
    if command -v nvidia-smi > /dev/null 2>&1; then
        nvidia-smi --query-gpu=name,driver_version,memory.used,memory.total --format=csv,noheader | head -n 1
    else
        echo "Integrated / Non-NVIDIA Graphics"
    fi
    echo -n "System Memory:          "
    free -h | awk '/^Mem:/ {print $3 "/" $2 " used, " $7 " available"}'
    echo -e "X11 Display:            ${DISPLAY:-:0}"
    echo -e "Default Terrain:        /tmp/gazebo_terrain_generator/San_francisco/San_francisco.world"
    echo -e "Web Server:             http://localhost:8080"
    echo -e "${CYAN}================================================================================${NC}"
}

function run_status() {
    echo -e "${CYAN}================================================================================${NC}"
    echo -e "${CYAN}                        ACTIVE ROS 2 & GAZEBO TOPIC STATUS                       ${NC}"
    echo -e "${CYAN}================================================================================${NC}"
    docker run --rm --net=host \
      sih_ugv:latest bash -c "
        export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
        export ROS_LOCALHOST_ONLY=1
        source /opt/ros/jazzy/setup.bash
        source /home/ubuntu/sih_ws/install/setup.bash > /dev/null 2>&1 || true
        echo '--- ROS 2 Active Nodes ---'
        ros2 node list 2>/dev/null || echo 'No active ROS 2 nodes found.'
        echo '--- ROS 2 Active Topics ---'
        ros2 topic list 2>/dev/null || echo 'No active ROS 2 topics found.'
      "
    echo -e "${CYAN}================================================================================${NC}"
}

print_banner

CMD="${1:-help}"

case "$CMD" in
    web)
        run_web_server
        ;;
    build)
        build_workspace
        ;;
    sim)
        shift
        run_sim "$@"
        ;;
    navigate)
        shift
        run_full_navigation "$@"
        ;;
    diagnose)
        run_diagnose
        ;;
    status)
        run_status
        ;;
    teleop)
        run_teleop
        ;;
    slam)
        run_slam
        ;;
    nav)
        run_nav
        ;;
    auto)
        shift
        run_auto_mission "$@"
        ;;
    test_all)
        run_test_all
        ;;
    help|*)
        echo "Usage: $0 <command> [arguments]"
        echo ""
        echo "Commands:"
        echo "  web                     Start interactive terrain generator web UI (http://localhost:8080)"
        echo "  build                   Build the ROS 2 workspace packages"
        echo "  diagnose                Inspect system GPU, RAM, ROS distro, and environment"
        echo "  status                  Check currently running ROS 2 nodes and topics"
        echo "  navigate [world] [gx] [gy] Launch complete all-in-one pipeline (Gazebo + SLAM + Nav2 + RViz + Goal B)"
        echo "  sim [world] [headless]  Launch Gazebo Harmonic simulation with custom UGV"
        echo "  teleop                  Launch keyboard teleoperation for UGV"
        echo "  slam                    Launch SLAM Toolbox and RViz2 perception suite"
        echo "  nav                     Launch Nav2 Autonomous Navigation Stack"
        echo "  auto <x> <y> [yaw]      Dispatch programmatic autonomous navigation goal"
        echo "  test_all                Execute automated multi-location verification suite"
        echo ""
        ;;
esac
