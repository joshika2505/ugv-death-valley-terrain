# UGV Belt Drive Robot — ROS 2 Package with FPV Camera POV Stream

Complete ROS 2 package for the **UGV Belt Drive Terrain Robot** equipped with **Prismatic Camera Mast Suspension**, **Skid-Steer Track Drive**, and **Live First-Person View (FPV / POV) Camera Feed**.

---

## 📁 Package Structure
```
ugv_belt_drive_ros2/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── camera_pov.rviz        # Pre-configured RViz 2 display with live /ugv/camera/image_raw stream
├── launch/
│   ├── display.launch.py       # Offline URDF & Joint State Publisher GUI inspection
│   ├── gazebo_pov.launch.py    # Main launch file: Gazebo World + UGV Spawners + Live FPV Camera POV
│   ├── rsp.launch.py           # Robot State Publisher
│   └── teleop.launch.py        # Keyboard teleop controller
├── meshes/
│   ├── camera_gimbal_head.stl
│   ├── camera_suspension_base.stl
│   ├── camera_suspension_piston.stl
│   ├── camera_suspension_spring.stl
│   ├── sprocket_wheel.stl
│   ├── track_belt.stl
│   └── ugv_chassis.stl
├── urdf/
│   └── ugv_belt_drive_robot.urdf.xacro   # Complete V3 UGV robot model
└── worlds/
    └── terrain_world.world     # Gazebo simulation world with ramps, slopes, & visual targets
```

---

## ⚙️ Quickstart Instructions (ROS 2 Humble / Iron / Rolling / Jazzy)

### 1. Build the Package
Copy `ugv_belt_drive_ros2` into your ROS 2 workspace `src` directory, then build with `colcon`:
```bash
cd ~/ros2_ws
colcon build --packages-select ugv_belt_drive_ros2
source install/setup.bash
```
*(On Windows CMD/PowerShell: `call install/setup.bat` or `install\setup.ps1`)*

---

### 2. Launch Full Gazebo World + FPV Camera POV Stream
To start Gazebo physics simulation, spawn the UGV robot, and view the **Live Camera POV Stream**:
```bash
ros2 launch ugv_belt_drive_ros2 gazebo_pov.launch.py
```

* **Live Camera Stream Topic**: `/ugv/camera/image_raw`
* **Camera IMU Topic**: `/ugv/camera/imu/data`
* **Chassis IMU Topic**: `/imu/data`

---

### 3. Drive the Robot (Keyboard Teleop)
In a separate terminal, launch keyboard teleop to drive the UGV tracks:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/ugv/cmd_vel
```
* Press `i` to drive forward.
* Press `k` to stop.
* Press `j` / `l` to turn left / right.
* Watch the live camera POV stream update in real-time as the robot moves across terrain ramps!

---

### 4. Inspect Robot URDF Joints Offline (No Gazebo)
To inspect the 3D model, joint limits, and suspension without launching Gazebo:
```bash
ros2 launch ugv_belt_drive_ros2 display.launch.py
```
