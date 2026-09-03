# Tracked UGV Perception, 3D Depth Distance & Traversability Classification Pipeline

A high-performance, real-time perception engine engineered for continuous-track Unmanned Ground Vehicles (UGVs) operating in unstructured off-road environments.

Combines **3D stereo RGB-D geometric reconstruction** with **4-class semantic scene segmentation** and a **physics-informed traversability classification engine** to enable safe, adaptive autonomous navigation.

---

## Key Features

1. **Precision 3D Metric Localization**: Vectorized back-projection from calibrated stereo camera depth to robot base coordinate frame $(X_r, Y_r, Z_r)$, computing exact radial distances $d = \sqrt{X^2 + Y^2}$ and azimuth bearings.
2. **Fast RANSAC Ground Surface Extraction**: Real-time ground plane fitting ($ax + by + cz + d = 0$) and local terrain slope estimation ($\theta = \arccos(\hat{n} \cdot \hat{z})$).
3. **Physics-Informed Traversability Engine**:
   - **Track Run-Over Allowed**: Distinguishes crushable vegetation (tall grass up to $40\text{ cm}$) and low rigid steps ($\le 15\text{ cm}$) from lethal obstacles.
   - **Lethal Obstacle Avoidance**: Flags boulders, tree trunks, and structures exceeding track step climb limit.
   - **Negative Hazard Detection**: Identifies drop-offs, trenches, and ditches ($> 12\text{ cm}$ step-down) with fatal avoidance cost.
4. **2.5D Local Grid Costmap**: Ego-centric $(160 \times 160)$ grid at $5\text{ cm/cell}$ with continuous track footprint inflation.
5. **3D Spatial Obstacle Detector**: Clusters elevated features into 3D bounding boxes with height, width, range, bearing, and run-over tags.
6. **ROS2 Drop-In Interfaces**: Native conversion to `nav_msgs/msg/OccupancyGrid` and `vision_msgs/msg/Detection3DArray`.
7. **Edge Optimized**: Designed for sub-$28\text{ ms}$ latency ($> 30\text{ FPS}$) on NVIDIA Jetson Orin / standard edge compute.

---

## System Architecture

```
+------------------------------------------------------------------------------------------------+
|                                    Stereo RGB-D Camera Stream                                  |
|                              (Synthetic Generator / Live Sensor / ROS2)                        |
+-----------------------------------------------+------------------------------------------------+
                                                |
                       +------------------------+------------------------+
                       |                                                 |
                       v                                                 v
           [ RGB Image (640x480x3) ]                         [ Depth Map (640x480) ]
                       |                                                 |
                       v                                                 v
        +------------------------------+                 +--------------------------------+
        |   Semantic Segmentation      |                 |    3D Metric Back-Projection   |
        |   (PIDNet / Feature-Based /  |                 |    & Ground Plane Fit (RANSAC) |
        |    4-Class Taxonomy)         |                 |    Output: (X_r, Y_r, Z_r, dH) |
        +--------------+---------------+                 +---------------+----------------+
                       |                                                 |
                       +-----------------------+-------------------------+
                                               |
                                               v
                             +-----------------------------------+
                             | Tracked Vehicle Traversability    |
                             | Classification Engine             |
                             | - Geometric climb check (H_step)  |
                             | - Semantic material verification  |
                             | - Negative obstacle detector      |
                             +-----------------+-----------------+
                                               |
                                               v
            +----------------------------------+----------------------------------+
            |                                                                     |
            v                                                                     v
+-------------------------------+                               +-----------------------------------+
|     2.5D Local Costmap        |                               |   3D Spatial Obstacle Extractor   |
|   - Multi-layer grid (0-254)  |                               |   - Spatial (X,Y,Z), Range, Bear  |
|   - Vehicle footprint inflate |                               |   - Classification & Run-over Tag |
+---------------+---------------+                               +-----------------+-----------------+
                |                                                                 |
                +-------------------------------+---------------------------------+
                                                |
                                                v
                             +-----------------------------------+
                             | ROS2 & Standalone Output Dispatch |
                             | - /perception/traversability_grid |
                             | - /perception/detected_obstacles  |
                             | - /perception/debug_overlay       |
                             +-----------------------------------+
```

---

## Semantic & Traversability Taxonomy

| Class ID | Semantic Label | Typical Terrain | Decision Verdict | Cost |
| :--- | :--- | :--- | :--- | :--- |
| **0** | `FREE_DRIVABLE` | Flat dirt trail, packed soil, asphalt | **Free Drivable** | `0` |
| **1** | `SOFT_TRAVERSABLE` | Tall grass, light weeds ($\le 40\text{ cm}$) | **Traversable (Run Over)** | `15` |
| **2** | `RIGID_OBSTACLE` | Low rock / curb ($\le 15\text{ cm}$) | **Traversable (Run Over)** | `25 - 80` |
| **2** | `RIGID_OBSTACLE` | Boulder, tree trunk ($> 15\text{ cm}$) | **Lethal Obstacle (Avoid)** | `254` |
| **3** | `NEGATIVE_HAZARD` | Ditch, drop-off, trench ($> 12\text{ cm}$) | **Negative Hazard (Avoid)** | `255` |

---

## Directory Structure

```
SIH-renumaa/
├── sih_perception/
│   ├── core/
│   │   ├── camera.py             # Stereo camera math & 3D back-projection
│   │   └── geometry.py           # RANSAC ground plane fitting & height differentials
│   ├── segmentation/
│   │   ├── taxonomy.py           # Functional 4-class taxonomy & color palette
│   │   └── segmenter.py          # Unified semantic segmentation backbones
│   ├── traversability/
│   │   ├── vehicle_profile.py    # Tracked robot physical envelope & clearances
│   │   ├── decision_engine.py    # Hybrid geometric-semantic decision logic
│   │   └── costmap.py            # 2.5D local grid costmap & obstacle inflation
│   ├── detection/
│   │   └── obstacle_detector.py  # 3D spatial obstacle clustering & metric localization
│   ├── pipeline/
│   │   ├── pipeline.py           # Unified real-time perception coordinator
│   │   └── ros2_bridge.py        # ROS2 Nav2 / Vision msg serialization
│   ├── simulation/
│   │   └── synthetic_scene.py    # 3D scene & stereo sensor data generator
│   └── visualization/
│       └── visualizer.py         # 4-Panel telemetry & diagnostic dashboard renderer
├── benchmarks/
│   └── benchmark_runner.py       # Latency percentiles & validation test suite
├── tests/
│   ├── test_camera.py            # Intrinsics/extrinsics & projection tests
│   ├── test_geometry.py          # Ground fitting & delta_h unit tests
│   ├── test_traversability.py    # Decision engine rule tests
│   └── test_costmap.py           # 2.5D costmap & inflation tests
├── config/
│   └── default_config.yaml       # Hardware & vehicle profile parameters
├── demo.py                       # Interactive scenario runner & visualizer
├── implementation_plan.md        # Detailed engineering specification
└── setup.py                      # Package installation script
```

---

## Quickstart Guide

### 1. Installation
```bash
# Clone and enter directory
cd /path/to/SIH-renumaa

# Setup Python environment
python3 -m venv venv
source venv/bin/activate
pip install -e .
```

### 2. Run Automated Test Suite
```bash
pytest tests/ -v
```

### 3. Run Interactive Multi-Scenario Demo
```bash
# Runs all 5 scenarios and outputs composite diagnostic dashboard images to outputs/
python demo.py --scenario all
```

### 4. Run Edge Performance Benchmarks
```bash
python benchmarks/benchmark_runner.py
```
