# Weapon Docking

USB camera weapon docking ROS 2 package migrated for `/home/rc3/tmp/pingmu_test`.

The node detects a custom ArUco screen tag, estimates the tag pose, publishes docking error, and converts the error into chassis velocity on `/cmd_vel`.

## Recognition Chain

```text
USB camera
  -> OpenCV ArUco custom dictionary
  -> 6x6 marker, 1000 IDs, seed 0
  -> target tag ID 56
  -> marker black-square side length 0.06 m
  -> solvePnP(SOLVEPNP_IPPE_SQUARE)
  -> x_error / y_error / z_error
  -> /cmd_vel
```

The custom dictionary generated in C++ by OpenCV with `generateCustomDictionary(1000, 6, 0)` matches `../aruco_custom_6x6_1000.npz`.

## Build

From `/home/rc3/tmp/pingmu_test`:

```bash
colcon build --packages-select weapon_docking --symlink-install
source install/setup.bash
```

## Launch

```bash
cd /home/rc3/tmp/pingmu_test
source install/setup.bash
ros2 launch weapon_docking weapon_docking.launch.py
```

Launch loads:

```text
weapon_docking/config/params.yaml
```

Important defaults:

```yaml
camera_index: 0
frame_width: 1280
frame_height: 800
frame_fps: 120
frame_fourcc: "MJPG"
enable_ui: true
fx: 956.80819
fy: 956.46353
cx: 653.72184
cy: 358.43826
dist_k1: -0.005408
dist_k2: 0.029448
dist_p1: -0.000804
dist_p2: -0.00107
dist_k3: 0.0
tag_size_m: 0.06
target_tag_id: 56
aruco_dictionary_size: 1000
aruco_marker_bits: 6
aruco_dictionary_seed: 0
```

## Action

Start docking with:

```bash
ros2 action send_goal --feedback /weapon_dock weapon_docking/action/WeaponDock "{task_complete: false, target_dist: 0.3}"
```

`target_dist` is the target camera-to-tag forward distance in meters. With `target_dist: 0.3`, the second stage stops when the estimated camera-to-tag distance is about 300 mm.

## Chassis Control Logic

The action execution loop runs at 30 Hz:

```text
detect target tag
  -> no valid target: publish zero /cmd_vel and wait
  -> valid target: publish /task/weapon_docking/error
  -> kVisualAlign: control lateral offset and yaw
  -> kOpenLoopX: control forward distance to target_dist
  -> kDone: publish zero /cmd_vel and finish action
```

Published topics:

```text
/cmd_vel
  geometry_msgs/msg/Twist
  uses linear.x, linear.y, angular.z

/task/weapon_docking/error
  std_msgs/msg/Float32MultiArray
  data[0] = x_error in mm
  data[1] = y_error in mm
  data[2] = z_error in mrad
```

## Runtime Check

Terminal 1:

```bash
cd /home/rc3/tmp/pingmu_test
source install/setup.bash
ros2 launch weapon_docking weapon_docking.launch.py
```

Terminal 2:

```bash
source /home/rc3/tmp/pingmu_test/install/setup.bash
ros2 topic echo /cmd_vel
```

Terminal 3:

```bash
source /home/rc3/tmp/pingmu_test/install/setup.bash
ros2 topic echo /task/weapon_docking/error
```

Terminal 4:

```bash
source /home/rc3/tmp/pingmu_test/install/setup.bash
ros2 action send_goal --feedback /weapon_dock weapon_docking/action/WeaponDock "{task_complete: false, target_dist: 0.3}"
```

When target tag 56 is visible, `/cmd_vel.linear.x`, `/cmd_vel.linear.y`, `/cmd_vel.angular.z`, and `/task/weapon_docking/error` should update with the visual error. When the tag is lost, the node publishes zero `/cmd_vel`.
