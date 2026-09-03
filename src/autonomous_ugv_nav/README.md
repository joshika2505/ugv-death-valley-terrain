# 🤖 Autonomous UGV Navigation Stack (Perception-to-Action Pipeline)

A production-grade, non-hardcoded **ROS 2** navigation stack designed for Unmanned Ground Vehicles (UGVs) operating in **GPS-denied, unstructured outdoor terrain under full EMCON (Emission Control)** constraints.

---

## 🏗️ System Architecture Overview

```
                          [ Stereo Depth PointCloud2 / YOLO Detections ]
                                                │
                                                ▼
                      ┌──────────────────────────────────────────────────┐
                      │ semantic_costmap_node                            │
                      │ • 2.5D Elevation Grid Voxelization (0.1m res)    │
                      │ • Slope Filter (Sobel arctan gradient)           │
                      │ • Roughness Filter (Height residual variance)    │
                      │ • Step Height Filter (Clearance thresholding)    │
                      │ • Semantic Friction / Lethal Stamping            │
                      │ • Exponential Obstacle Inflation Buffer          │
                      └─────────────────────────┬────────────────────────┘
                                                │
                               /ugv/semantic_costmap (10 Hz)
                                                │
                         ┌──────────────────────┴──────────────────────┐
                         ▼                                             ▼
      ┌──────────────────────────────────────┐     ┌──────────────────────────────────────┐
      │ global_planner_node                  │     │ mppi_controller_node                 │
      │ • Weighted A* Heuristic Search       │     │ • SIMD Vectorized Batch Rollouts     │
      │ • Continuous Traversability Penalties│     │ • K = 300 Trajectories, T = 20 Steps │
      │ • Replans Corridor @ 1 Hz            │     │ • Skid-Steer Kinodynamics Model      │
      └──────────────────┬───────────────────┘     │ • Modular Cost Critics               │
                         │                         │   - Obstacle Proximity & Lethal      │
                         │ /ugv/global_plan        │   - Path Follow & Path Align         │
                         │                         │   - Goal & Terminal Angle            │
                         └────────────────────────►│   - Smoothness & Semantic Speed      │
                                                   └──────────────────┬───────────────────┘
                                                                      │ /cmd_vel (12 Hz)
                                                                      ▼
      ┌──────────────────────────────────────┐     ┌──────────────────────────────────────┐
      │ ekf_state_estimator_node             │     │ safety_monitor_node (Watchdog)       │
      │ • 5-State EKF [x, y, theta, v, omega]│     │ • FSM: NAVIGATE / RECOVERY / STOP    │
      │ • VIO + IMU + Wheel Encoders         │◄────┤ • /cmd_vel Frequency Monitor (<5 Hz) │
      │ • Slip Detector (1e6 Covariance Gate)│     │ • Feature Starvation Monitor (<50 pts│
      └──────────────────────────────────────┘     └──────────────────────────────────────┘
```

---

## 📂 Package Directory Structure

```
autonomous_ugv_nav/
├── package.xml                       # ROS 2 package manifest
├── setup.py                          # Python setup & entry point bindings
├── setup.cfg                         # Package executable install path
├── README.md                         # Full documentation and usage guide
├── resource/autonomous_ugv_nav       # Ament index package registration
│
├── config/
│   ├── nav2_params.yaml              # Nav2-compatible parameter file
│   ├── ekf_params.yaml               # EKF state estimator and slip gating config
│   ├── mppi_params.yaml              # MPPI trajectory optimizer tuning
│   └── costmap_params.yaml           # 2.5D Traversability and elevation thresholds
│
├── launch/
│   ├── ugv_navigation.launch.py      # Master bringup: EKF, Costmap, Planner, MPPI, Safety
│   └── perception_bridge.launch.py   # Perception → Navigation bridge & remap
│
├── autonomous_ugv_nav/
│   ├── __init__.py
│   │
│   ├── costmap/
│   │   ├── __init__.py
│   │   ├── costmap_types.py          # Standard cost values & semantic class mappings
│   │   ├── traversability_analyzer.py# Pure NumPy slope, roughness, and step-height filters
│   │   └── semantic_costmap_node.py  # PointCloud2 + Detections -> OccupancyGrid ROS 2 node
│   │
│   ├── planner/
│   │   ├── __init__.py
│   │   ├── skid_steer_model.py       # Differential-drive / skid-steer 2nd-order Runge-Kutta kinematics
│   │   ├── cost_critics.py           # Modular critics (Obstacle, Path, Goal, Smoothness, Speed)
│   │   ├── mppi_core.py              # Vectorized NumPy SIMD MPPI controller
│   │   ├── mppi_controller_node.py   # ROS 2 local planner node publishing /cmd_vel
│   │   └── global_planner_node.py    # Weighted A* on continuous traversability costmap
│   │
│   ├── estimator/
│   │   ├── __init__.py
│   │   ├── slip_detector.py          # Velocity discrepancy monitor & covariance scaler
│   │   ├── ekf_core.py               # 5-State EKF [x, y, theta, v, omega]
│   │   └── ekf_state_estimator_node.py # 30 Hz ROS 2 node fusing VIO, IMU, and Encoders
│   │
│   └── safety/
│       ├── __init__.py
│       ├── behavior_state_machine.py # FSM: NAVIGATING, FEATURE_RECOVERY, SAFE_STOP
│       └── safety_monitor_node.py    # Planner frequency, thermal & starvation watchdog
│
└── test/
    ├── test_mppi_core.py             # Vectorized rollout & cost weighting tests
    ├── test_ekf_core.py              # EKF predict/update & slip gating tests
    ├── test_traversability.py        # Slope, roughness, and step height filter tests
    └── test_cost_critics.py          # Individual critic evaluation tests
```

---

## 🚀 Quickstart & Build Instructions

### 1. Prerequisites
- **ROS 2**: Jazzy / Iron / Humble
- **Python**: 3.10+
- **Packages**: `numpy`, `scipy` (optional, pure numpy fallback supported), `pytest`

### 2. Building with Colcon
From your ROS 2 workspace root (`~/ros2_ws`):
```bash
colcon build --packages-select autonomous_ugv_nav
source install/setup.bash
```

### 3. Running the Automated Unit Test Suite
Run the 16 unit tests (which execute completely independently of ROS 2 middleware):
```bash
python3 -m pytest src/autonomous_ugv_nav/test/ -v
# or
colcon test --packages-select autonomous_ugv_nav && colcon test-result --all
```

### 4. Launching the Master Navigation Pipeline
```bash
ros2 launch autonomous_ugv_nav ugv_navigation.launch.py \
    use_sim_time:=false \
    pointcloud_topic:=/oak/points
```

---

## 📡 Topic Interfaces

| Topic Name | Message Type | Rate | Description |
| :--- | :--- | :--- | :--- |
| `/oak/points` | `sensor_msgs/PointCloud2` | 30 Hz | Input stereo depth point cloud |
| `/imu/data` | `sensor_msgs/Imu` | 200 Hz | Input 6-DOF IMU acceleration & angular velocity |
| `/odom` | `nav_msgs/Odometry` | 50 Hz | Input wheel encoder odometry |
| `/visual_slam/odom` | `nav_msgs/Odometry` | 30 Hz | Input visual-inertial odometry |
| `/ugv/odom_filtered` | `nav_msgs/Odometry` | 30 Hz | Output fused slip-robust state estimate |
| `/ugv/semantic_costmap` | `nav_msgs/OccupancyGrid` | 10 Hz | Output 2.5D traversability costmap |
| `/ugv/global_plan` | `nav_msgs/Path` | 1 Hz | Output Weighted A* global route corridor |
| `/cmd_vel` | `geometry_msgs/Twist` | 12 Hz | Output skid-steer velocity commands |
| `/ugv/diagnostics/slip` | `std_msgs/Bool` | 30 Hz | Diagnostic wheel slip alert |
| `/ugv/safety/state` | `std_msgs/String` | 20 Hz | Current watchdog safety state |
