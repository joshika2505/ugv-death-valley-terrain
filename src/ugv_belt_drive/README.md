# ugv_belt_drive

Belt-drive (tracked) terrain UGV — ROS 2 + Gazebo Classic package.
Mobility chain: front driven sprocket + rear idler sprocket per side,
joined by a visual/collision belt shell, commanded together as a
skid-steer pair via `libgazebo_ros_diff_drive`.

Assumes **ROS 2 Humble + Gazebo Classic (11)**. If you're on Jazzy /
Gazebo Sim (Harmonic) instead, tell me and I'll port the plugin tags
(`ros_gz_sim` uses different plugin names than `gazebo_ros_pkgs`).

## Build

```bash
mkdir -p ~/ugv_ws/src
cp -r ugv_belt_drive ~/ugv_ws/src/
cd ~/ugv_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run: spawn the robot on terrain

```bash
ros2 launch ugv_belt_drive spawn_robot.launch.py
```

This starts Gazebo with `terrain_world.world` (ramps, rocks, a trench
gap) and spawns the robot via `robot_state_publisher` +
`spawn_entity.py`.

## Drive it (belt/skid-steer mobility)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/ugv/cmd_vel
```

All four sprockets (front+rear, both sides) are commanded together per
side — that's the belt-drive mobility chain. `i/,` = forward/back,
`j/l` = turn in place (skid-steer pivot), same as any diff-drive robot.

## SLAM + visualize

In a second terminal (after the robot has spawned):

```bash
ros2 launch ugv_belt_drive slam.launch.py
```

Opens `slam_toolbox` (mapping mode, subscribed to `/scan`) and RViz
with robot model, TF, LiDAR scan, the live rover-camera feed, and the
growing map — all pre-wired in `rviz/ugv_view.rviz`.

## Pan-tilt camera (manual test)

```bash
ros2 topic pub /camera_pan_joint_position std_msgs/msg/Float64 "data: 0.5" --once
```

(Wire this to a real controller — e.g. `joint_trajectory_controller` —
if you want closed-loop pan/tilt control instead of one-shot topic
pokes; happy to add that controller config if you want it.)

## Known things to verify on first real run

- `num_wheel_pairs=2` with repeated `left_joint`/`right_joint` tags on
  `libgazebo_ros_diff_drive` is the documented way to drive a
  multi-axle/tracked skid-steer rig in `gazebo_ros_pkgs`, but plugin
  behavior can vary slightly by distro patch version — test a straight
  drive + a pivot turn first before trusting odometry.
- The belt is a simplified box+cylinder stand-in, not a real
  continuous-track contact model — expect it to behave like a
  4-wheel skid-steer, not a true tracked vehicle, over the trench gap.
- Camera pan/tilt joints have no controller wired yet (see above) —
  they'll sit at zero until you either publish joint commands or add
  a trajectory controller.
