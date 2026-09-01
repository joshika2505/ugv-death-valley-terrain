#!/usr/bin/env bash
# ==============================================================================
# Master Execution & Demonstration Script for Forest UGV Autonomous Navigation
# Vision-Only GPS-Denied Autonomous Navigation System (ROS 2 + Gazebo Sim)
# ==============================================================================

set -e

CONTAINER_NAME="sih_ugv_runner"
WS_PATH="/home/ubuntu/sih_ws"
SRC_PATH="/home/joshika/Desktop/SIH"
MODE="${1:-mission}"
export GEMINI_API_KEY="${GEMINI_API_KEY:-YOUR_API_KEY_HERE}"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BOLD='\033[1m'
NC='\033[0m'

echo -e "${BLUE}${BOLD}==========================================================${NC}"
echo -e "${GREEN}${BOLD}  🌲 FOREST UGV: VISION-ONLY GPS-DENIED AUTONOMY 🌲       ${NC}"
echo -e "${BLUE}${BOLD}  Operational Mode: ${YELLOW}${MODE}${NC}"
echo -e "${BLUE}${BOLD}==========================================================${NC}"

# Allow X11 display forwarding
xhost +local: > /dev/null 2>&1 || true

# Function to ensure docker runner container is running
ensure_container() {
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo -e "${YELLOW}Starting Docker container ${CONTAINER_NAME}...${NC}"
        docker start ${CONTAINER_NAME} > /dev/null 2>&1 || {
            docker run -d \
                --name ${CONTAINER_NAME} \
                --net=host \
                --privileged \
                --ipc=host \
                -e DISPLAY="${DISPLAY:-:0}" \
                -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                -v /dev:/dev \
                -v "${SRC_PATH}":"${WS_PATH}/src":rw \
                sih_ugv:latest \
                sleep infinity
        }
    fi
}

# Function to build workspace
build_workspace() {
    echo -e "${BLUE}Building Forest UGV ROS 2 packages with colcon...${NC}"
    docker exec -it ${CONTAINER_NAME} bash -c "\
        source /opt/ros/jazzy/setup.bash && \
        cd ${WS_PATH} && \
        colcon build"
    echo -e "${GREEN}Build complete!${NC}"
}

case "$MODE" in
    build)
        ensure_container
        build_workspace
        ;;

    mission|full)
        ensure_container
        build_workspace
        echo -e "${GREEN}Launching GPS-Denied Autonomous Forest Mission (Point A -> Point B)...${NC}"
        docker exec -i -e GEMINI_API_KEY="${GEMINI_API_KEY}" ${CONTAINER_NAME} bash -c "\
            source /opt/ros/jazzy/setup.bash && \
            source ${WS_PATH}/install/setup.bash && \
            ros2 launch forest_ugv_bringup mission.launch.py scenario:=forest_world gps_enabled:=false enable_gemini:=true rviz:=true"
        ;;

    gemini)
        ensure_container
        build_workspace
        echo -e "${GREEN}Launching Autonomous Forest Mission with Google Gemini Multimodal VLA Brain...${NC}"
        docker exec -i -e GEMINI_API_KEY="${GEMINI_API_KEY}" ${CONTAINER_NAME} bash -c "\
            source /opt/ros/jazzy/setup.bash && \
            source ${WS_PATH}/install/setup.bash && \
            ros2 launch forest_ugv_bringup mission.launch.py scenario:=forest_world gps_enabled:=false enable_gemini:=true rviz:=true"
        ;;

    extreme|hardcore|max)
        ensure_container
        build_workspace
        echo -e "${RED}${BOLD}Launching EXTREME HARDCORE Forest Mission (MAX DIFFICULTY: 4x Fallen Trees, Boulder Fields, Ditches)...${NC}"
        docker exec -i -e GEMINI_API_KEY="${GEMINI_API_KEY}" ${CONTAINER_NAME} bash -c "\
            source /opt/ros/jazzy/setup.bash && \
            source ${WS_PATH}/install/setup.bash && \
            ros2 launch forest_ugv_bringup mission.launch.py scenario:=forest_extreme_hardcore gps_enabled:=false enable_gemini:=true rviz:=true"
        ;;

    scenario)
        SCENARIO_NUM="${2:-1}"
        case "$SCENARIO_NUM" in
            1|open) SCENARIO_NAME="forest_open_trail" ;;
            2|rocky) SCENARIO_NAME="forest_rocky" ;;
            3|fallen_tree) SCENARIO_NAME="forest_fallen_tree" ;;
            4|ditch|slope) SCENARIO_NAME="forest_ditch_slope" ;;
            5|dynamic) SCENARIO_NAME="forest_dynamic_obstacle" ;;
            6|extreme|max) SCENARIO_NAME="forest_extreme_hardcore" ;;
            *) SCENARIO_NAME="forest_world" ;;
        esac
        ensure_container
        build_workspace
        echo -e "${GREEN}Launching Scenario: ${SCENARIO_NAME}...${NC}"
        docker exec -i -e GEMINI_API_KEY="${GEMINI_API_KEY}" ${CONTAINER_NAME} bash -c "\
            source /opt/ros/jazzy/setup.bash && \
            source ${WS_PATH}/install/setup.bash && \
            ros2 launch forest_ugv_bringup mission.launch.py scenario:=${SCENARIO_NAME} gps_enabled:=false enable_gemini:=true rviz:=true"
        ;;

    dynamic_obstacle)
        ensure_container
        build_workspace
        echo -e "${GREEN}Launching Dynamic Obstacle Scenario & Avoidance Replanner...${NC}"
        docker exec -it ${CONTAINER_NAME} bash -c "\
            source /opt/ros/jazzy/setup.bash && \
            source ${WS_PATH}/install/setup.bash && \
            ros2 launch forest_ugv_bringup dynamic_obstacle.launch.py"
        ;;

    teleop)
        ensure_container
        echo -e "${YELLOW}Starting Keyboard Teleop Node for Forest UGV...${NC}"
        docker exec -it ${CONTAINER_NAME} bash -c "\
            source /opt/ros/jazzy/setup.bash && \
            ros2 run teleop_twist_keyboard teleop_twist_keyboard"
        ;;

    test)
        ensure_container
        build_workspace
        echo -e "${BLUE}Running Forest System Test Suite...${NC}"
        docker exec -it ${CONTAINER_NAME} bash -c "\
            source /opt/ros/jazzy/setup.bash && \
            source ${WS_PATH}/install/setup.bash && \
            python3 ${WS_PATH}/src/test_forest_system.py"
        ;;

    experiments|eval)
        ensure_container
        build_workspace
        echo -e "${GREEN}Executing Experiment Benchmarks (GPS ON vs GPS OFF & Lighting Tests)...${NC}"
        docker exec -it ${CONTAINER_NAME} bash -c "\
            source /opt/ros/jazzy/setup.bash && \
            source ${WS_PATH}/install/setup.bash && \
            python3 ${WS_PATH}/src/run_experiments.py"
        ;;

    dashboard)
        ensure_container
        build_workspace
        echo -e "${GREEN}Launching HERCULES Mission Control Dashboard on http://localhost:8080...${NC}"
        docker exec -it ${CONTAINER_NAME} bash -c "\
            source /opt/ros/jazzy/setup.bash && \
            source ${WS_PATH}/install/setup.bash && \
            ros2 launch forest_dashboard dashboard.launch.py"
        ;;

    stop)
        echo -e "${RED}Stopping all Gazebo Sim, ROS 2, and Dashboard processes...${NC}"
        docker exec ${CONTAINER_NAME} pkill -9 -f "forest_|gemini_|dashboard_|ros2|gz|gazebo|rviz2|parameter_bridge" || true
        echo -e "${GREEN}Processes cleanly terminated.${NC}"
        ;;

    *)
        echo -e "${RED}Unknown mode: ${MODE}${NC}"
        echo "Usage: $0 {mission|scenario <1-5>|dashboard|dynamic_obstacle|teleop|test|experiments|build|stop}"
        exit 1
        ;;
esac
