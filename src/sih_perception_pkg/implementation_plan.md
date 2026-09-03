# Course of Action: Perception, Depth Distance & Traversability Classification Pipeline

## Overview & Objective
This course of action defines the end-to-end engineering workflow for building the **Perception Model** on an Unmanned Ground Vehicle (UGV) equipped with continuous tracks and a stereo RGB-D camera.

The model is responsible for:
1. **Detection & Spatial Localization**: Identifying terrain features/objects in front of the bot and calculating precise real-world 3D distances $(X, Y, Z)$ using stereo depth.
2. **Tracked Traversability Classification**: Determining whether an object/surface is a **Traversable Feature** (can be driven over by tracks, e.g., low grass, small debris, gravel mounds $\le$ ground clearance) or a **Lethal Obstacle** (must be avoided, e.g., tree trunks, boulders, walls, steep drop-offs/ditches).

---

## 1. System Architecture: Hybrid Semantic + 3D Geometric Perception

Relying solely on 2D bounding boxes (e.g., YOLO) cannot tell the physical height or volume of a rock to know if tracks can clear it. Conversely, relying solely on raw depth cannot tell whether a 20 cm height profile is soft deformable grass (run-over) or a rigid concrete curb (avoid).

```
+-----------------------------------------------------------------------------------+
|                            Stereo RGB-D Camera                                    |
+------------------------------------+----------------------------------------------+
                                     |
                  +------------------+-------------------+
                  |                                      |
                  v                                      v
        [ Left RGB Stream ]                    [ Stereo Depth Stream ]
                  |                                      |
                  v                                      v
  +-------------------------------+             +---------------------------------+
  | Real-Time Semantic Segmenter  |             |  3D Back-Projection & Ground    |
  | (PIDNet-S / MobileNetV4-Seg)  |             |  Plane Extraction (RANSAC)      |
  | Output: Per-pixel Class Mask  |             |  Output: 3D Point Cloud (X,Y,Z) |
  +---------------+---------------+             +----------------+----------------+
                  |                                              |
                  +----------------------+-----------------------+
                                         |
                                         v
                 +-----------------------------------------------+
                 | Tracked-Vehicle Traversability Decision Logic |
                 |  - Geometric Clearance Check: h_obs <= H_max  |
                 |  - Semantic Material Verification             |
                 |  - Positive / Negative Obstacle Extractor     |
                 +-----------------------+-----------------------+
                                         |
                                         v
                 +-----------------------------------------------+
                 |           Outputs to Reactive Planner         |
                 |  1. Obstacle Array (Distances, Angles, Types) |
                 |  2. 2.5D Local Traversability Costmap         |
                 +-----------------------------------------------+
```

---

## 2. Phase-by-Phase Course of Action

### Phase 1: Problem Formulation & Tracked Vehicle Physics Profile
Define the physical envelope of the tracked robot to establish crisp numerical thresholds for the decision model:
* **Ground Clearance ($H_{\text{clearance}}$)**: e.g., $8\text{ to }12\text{ cm}$.
* **Maximum Step/Obstacle Climb Height ($H_{\text{step\_max}}$)**: e.g., $15\text{ cm}$ (tracked vehicles can typically climb obstacles higher than wheeled chassis by engaging track treads).
* **Maximum Slope Angle ($\theta_{\text{max}}$)**: e.g., $30^\circ\text{ to }35^\circ$.
* **Trench / Ditch Width Limit ($W_{\text{trench\_max}}$)**: Span tracks can cross without nose-diving.
* **Semantic Class Taxonomy**:
  * `Class 0 (Free Drivable)`: Flat ground, dirt trail, pavement, packed soil.
  * `Class 1 (Soft Traversable)`: Tall grass, light weeds, small twigs (tracks crush without resistance).
  * `Class 2 (Rigid Obstacle - Avoid)`: Rocks $> H_{\text{step\_max}}$, tree trunks, posts, humans, vehicles, solid walls.
  * `Class 3 (Negative Obstacle / Hazard - Avoid)`: Ditches, cliffs, drop-offs, deep water/mud.

---

### Phase 2: Dataset Preparation & Semantic Model Training
1. **Dataset Selection**:
   * Leverage established off-road datasets: **RELLIS-3D**, **RUGD**, or **WildScenes**.
   * Re-map the granular annotations into the 4 functional categories defined above.
2. **Model Architecture Selection**:
   * Use high-fps, edge-optimized segmentation backbones: **PIDNet-S** (Proportional-Integral-Derivative Network for real-time semantic segmentation) or **MobileNetV4-SegFormer**.
   * Focus training loss (Focal Loss / OHEM) on boundary precision and negative obstacles (preventing false negatives on ditches).
3. **Training & Validation Metrics**:
   * Mean Intersection over Union (mIoU) on `Rigid Obstacle` and `Negative Obstacle` classes.
   * Precision threshold target $> 95\%$ on lethal obstacles (prioritize zero false-negatives for hazards).

---

### Phase 3: 3D Geometric Depth Extraction & Distance Calculation
1. **Camera Calibration & Rectification**:
   * Establish precise camera intrinsic matrix $K$:
     $$K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$$
   * Calibrate extrinsic transform $T_{\text{camera}}^{\text{base\_link}}$ to measure ground truth relative to robot ground contact.
2. **Back-Projection to 3D Metric Coordinates**:
   * For every pixel $(u, v)$ with depth $Z = D(u, v)$ (in meters):
     $$X_c = \frac{(u - c_x) \cdot Z}{f_x}, \quad Y_c = \frac{(v - c_y) \cdot Z}{f_y}, \quad Z_c = Z$$
   * Transform $(X_c, Y_c, Z_c) \xrightarrow{T_{\text{camera}}^{\text{base\_link}}} (X_r, Y_r, Z_r)$ in the robot's base frame ($X_r$: forward, $Y_r$: lateral, $Z_r$: height above ground).
3. **Distance Calculation**:
   * Radial distance to detected object centroid / boundary:
     $$d = \sqrt{X_r^2 + Y_r^2}$$
   * Bounding 3D cylinder or convex hull encapsulating the object.

---

### Phase 4: Fusion & Traversability Decision Engine
Implement the core decision algorithm combining geometry and semantic classification:

1. **Ground Plane Fitting (Local RANSAC / Patchwork++)**:
   * Fit the ground surface plane $P: ax + by + cz + d = 0$ directly in front of the tracks.
2. **Height & Gradient Differential**:
   * Compute the relative height $\Delta h(u, v) = Z_r(u, v) - Z_{\text{ground}}(X_r, Y_r)$.
   * Compute local surface gradient $\nabla Z$.
3. **Decision Rules**:
   * **Rule 1 (Hard Obstacle)**: If $\text{SemanticClass} \in \{\text{Rigid Obstacle}\}$ AND $\Delta h > H_{\text{step\_max}} \implies$ **STOP / AVOID** (Lethal cost).
   * **Rule 2 (Run-Over Allowed)**: If $\Delta h \le H_{\text{step\_max}}$ AND $\text{SemanticClass} \in \{\text{Free Drivable}, \text{Soft Traversable}\} \implies$ **TRAVERSABLE / RUN OVER** (Low cost).
   * **Rule 3 (Soft Tall Obstacle Override)**: If $\Delta h > H_{\text{step\_max}}$ BUT $\text{SemanticClass} == \text{Soft Traversable}$ (e.g., $30\text{ cm}$ light grass) $\implies$ **RUN OVER** (Moderate friction penalty, but safe).
   * **Rule 4 (Negative Obstacle / Ditch)**: If $\Delta h < -H_{\text{drop\_max}}$ (ground drops away) $\implies$ **LETHAL HAZARD / AVOID**.

---

### Phase 5: Edge Optimization & TensorRT Deployment on Jetson Orin
1. **Model Optimization**:
   * Export PyTorch trained weights $\to$ **ONNX** $\to$ **NVIDIA TensorRT** with FP16 precision.
   * Target inference latency on Jetson Orin Nano/NX: $\le 12\text{ ms}$ per frame.
2. **GPU Zero-Copy Pipeline**:
   * Ingest stereo frames directly into CUDA memory buffers.
   * Perform depth back-projection and semantic-depth masking using CUDA kernels or NVIDIA VPI (Vision Programming Interface).
3. **Execution Budget (30 Hz / 33 ms frame budget)**:
   * Frame Capture & Rectification: $\sim 5\text{ ms}$
   * TensorRT Segmentation: $\sim 12\text{ ms}$
   * 3D Depth Backprojection & Plane Fit: $\sim 8\text{ ms}$
   * Traversability Evaluation & Output Dispatch: $\sim 3\text{ ms}$
   * **Total Pipeline Latency**: $\sim 28\text{ ms}$ ($> 30\text{ FPS}$ real-time throughput).

---

### Phase 6: ROS2 Output Interface
Publish standard ROS2 messages for the local planner:
* `/perception/traversability_grid` (`nav_msgs/OccupancyGrid` or costmap layer).
* `/perception/detected_obstacles` (`vision_msgs/Detection3DArray` or custom message containing `[id, distance_m, azimuth_deg, height_m, is_run_over_allowed, class_name]`).
* `/perception/debug_overlay` (`sensor_msgs/Image` with colored traversability contours: Green = run over, Red = avoid, Yellow = caution).

---

## 3. Verification & Validation Milestones

### Milestone 1: Simulation & Offline Testing
* Validate depth-to-3D projection using recorded stereo ROSbags containing known obstacle sizes (e.g., $5\text{ cm}, 10\text{ cm}, 20\text{ cm}, 30\text{ cm}$ obstacles).
* Confirm distance accuracy within $\pm 5\text{ cm}$ up to $5\text{ meters}$.

### Milestone 2: Edge Hardware Benchmark (Jetson Orin)
* Validate TensorRT FP16 engine latency and memory consumption under full load.
* Verify thermal stability and sustained $\ge 25\text{ FPS}$.

### Milestone 3: Controlled Field Trials
* **Test Case A (Small debris / grass)**: UGV runs over $5\text{ cm}$ branches and tall grass without stopping.
* **Test Case B (Lethal step / rock)**: UGV detects $25\text{ cm}$ rock at $3\text{m}$, outputs non-traversable flag, and stops/reroutes.
* **Test Case C (Negative obstacle / ditch)**: UGV halts at simulated step-down / trench $> 10\text{ cm}$ deep.
