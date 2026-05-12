# Autonomous Wheel-Changing Simulation Using UR10e, Gazebo, OpenCV, TF, and MoveIt 2

This repository contains a ROS 2 Jazzy and Gazebo simulation for an autonomous wheel-changing task using a UR10e robot. The system uses a custom wheel-changing end-effector, an RGB-D camera, OpenCV-based wheel detection, TF transformation, and MoveIt 2 inverse kinematics.

The full simulation performs old-wheel removal, lug-nut removal, old-wheel dumping, new-wheel detection, MoveIt-based new-wheel approach, new-wheel gripping, wheel installation, lug-nut reattachment, and return-to-home motion.

---

## Main Features

- UR10e robot wheel-changing simulation in Gazebo
- Custom coaxial wheel-changing end-effector
- Outer wheel-gripping hands
- Inner socket tool for lug-nut handling
- Socket slide, socket spin, and socket finger joints
- End-effector RGB-D camera
- OpenCV-based new-wheel detection
- TF camera-to-base transformation
- MoveIt 2 inverse kinematics for new-wheel approach
- Gazebo detachable joints for wheel and lug-nut attachment
- Full autonomous task sequence script

---

## Repository Structure

```text
wheel-changing-autonomous-sim/
├── README.md
├── .gitignore
├── scripts/
│   └── run_old_dump_then_auto_new_grab.sh
└── src/
    ├── wheel_changing_description/
    ├── wheel_changing_gazebo/
    ├── wheel_changing_moveit_config/
    └── wheel_changing_vision_ik/
```

---

## Required System

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim
- MoveIt 2

---

## Install Dependencies

Run this once before building:

```bash
sudo apt update

sudo apt install -y \
  git \
  python3-colcon-common-extensions \
  python3-opencv \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-moveit \
  ros-jazzy-tf2-tools \
  ros-jazzy-rqt-image-view \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-image-geometry \
  ros-jazzy-joint-state-publisher-gui
```

---

## Build the Workspace

From the repository root:

```bash
cd ~/wheel-changing-autonomous-sim

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

Check that the packages are available:

```bash
ros2 pkg list | grep -E "wheel_changing_description|wheel_changing_gazebo|wheel_changing_moveit_config|wheel_changing_vision_ik"
```

Expected packages:

```text
wheel_changing_description
wheel_changing_gazebo
wheel_changing_moveit_config
wheel_changing_vision_ik
```

---

## Run the Full Autonomous Simulation

Use five terminals.

---

### Terminal 1: Start Gazebo

```bash
cd ~/wheel-changing-autonomous-sim

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH

ros2 launch wheel_changing_gazebo gazebo_ur10e_wheel_changer.launch.py
```

Wait until Gazebo opens and the robot/world appear.

---

### Terminal 2: Start Camera Bridge

The OpenCV detector is a ROS 2 node, but the camera is generated in Gazebo. Therefore, the RGB image, depth image, and camera info must be bridged from Gazebo to ROS 2.

```bash
source /opt/ros/jazzy/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /ee_rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /ee_rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image \
  /ee_rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

Keep this terminal running.

To confirm the camera topics are available in ROS 2:

```bash
ros2 topic list | grep -Ei "ee_rgbd|camera|image|depth|camera_info"
```

Expected topics:

```text
/ee_rgbd_camera/image
/ee_rgbd_camera/depth_image
/ee_rgbd_camera/camera_info
```

---

### Terminal 3: Start MoveIt move_group Only

Use the move-group-only launch file to avoid duplicate `robot_state_publisher` nodes.

```bash
cd ~/wheel-changing-autonomous-sim

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch wheel_changing_moveit_config demo_ompl_only_movegroup_only.launch.py
```

---

### Terminal 4: Start OpenCV Wheel Detector

```bash
cd ~/wheel-changing-autonomous-sim

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run wheel_changing_vision_ik wheel_3d_detector --ros-args \
  -p camera_frame:=ee_camera_optical_frame \
  -p dark_v_max:=120 \
  -p min_area:=300.0
```

At the beginning, the detector may show TF warnings. This is normal before the robot reaches the new-wheel detection pose. The full task script starts a joint-state TF stream at the correct detection pose.

---

### Terminal 5: Run the Full Autonomous Sequence

```bash
cd ~/wheel-changing-autonomous-sim

source /opt/ros/jazzy/setup.bash
source install/setup.bash

chmod +x scripts/run_old_dump_then_auto_new_grab.sh

./scripts/run_old_dump_then_auto_new_grab.sh
```

---

## Full Task Sequence

The full sequence performs:

1. Move to old wheel.
2. Grip old wheel with outer hands.
3. Extend socket to lug nut.
4. Grip lug nut with socket fingers.
5. Spin socket to remove lug nut.
6. Retract socket while holding lug nut.
7. Detach old wheel from stand.
8. Pull old wheel away from stand.
9. Move old wheel to dumping zone.
10. Release old wheel.
11. Move to new-wheel detection pose.
12. Start TF joint-state stream.
13. Detect new wheel using OpenCV.
14. Transform detected wheel point to `base_link`.
15. Use MoveIt 2 IK to compute new-wheel grabbing pose.
16. Move robot to new wheel.
17. Extend and close outer hands.
18. Detach new wheel from supply zone.
19. Carry new wheel to stand.
20. Attach new wheel to stand.
21. Place lug nut on new wheel.
22. Attach lug nut to new wheel.
23. Release socket and outer hands.
24. Return robot to home position.

---

## Important Camera Topics

```text
/ee_rgbd_camera/image
/ee_rgbd_camera/depth_image
/ee_rgbd_camera/camera_info
```

---

## Important Detector Topics

```text
/wheel_detector/annotated_image
/wheel_detector/new_wheel_center_px
/wheel_detector/new_wheel_point_camera
/wheel_detector/new_wheel_point_base
```

The most important detector output is:

```text
/wheel_detector/new_wheel_point_base
```

This topic gives the detected new-wheel position in the robot base frame.

---

## Important TF Frames

```text
base_link
wrist_3_link
ee_camera_link
ee_camera_optical_frame
```

The important TF chain is:

```text
base_link
→ wrist_3_link
→ tool0
→ ee_camera_link
→ ee_camera_optical_frame
```

---

## Optional: View Camera and Detector Output

To view the raw RGB camera image:

```bash
source /opt/ros/jazzy/setup.bash
source ~/wheel-changing-autonomous-sim/install/setup.bash

ros2 run rqt_image_view rqt_image_view /ee_rgbd_camera/image
```

To view the annotated detector image:

```bash
source /opt/ros/jazzy/setup.bash
source ~/wheel-changing-autonomous-sim/install/setup.bash

ros2 run rqt_image_view rqt_image_view /wheel_detector/annotated_image
```

---

## Useful Debug Commands

Check camera topics:

```bash
ros2 topic list | grep -Ei "ee_rgbd|camera|image|depth|camera_info"
```

Check detector topics:

```bash
ros2 topic list | grep wheel_detector
```

Check detector output:

```bash
ros2 topic echo /wheel_detector/new_wheel_point_base --once
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo base_link ee_camera_optical_frame
```

Check MoveIt node:

```bash
ros2 node list | grep move_group
```

Check TF tree:

```bash
ros2 run tf2_tools view_frames
```

This generates a `frames.pdf` file.

---

## Troubleshooting

### Problem: `auto_move_to_wheel_grab` waits for `/wheel_detector/new_wheel_point_base`

Cause: The OpenCV detector is not publishing the wheel point.

Fix:

1. Make sure the camera bridge is running.
2. Restart the wheel detector.
3. Check the detector image using `rqt_image_view`.
4. Confirm TF works using:

```bash
ros2 run tf2_ros tf2_echo base_link ee_camera_optical_frame
```

---

### Problem: Camera topics are missing

Start the bridge:

```bash
source /opt/ros/jazzy/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /ee_rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /ee_rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image \
  /ee_rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

---

### Problem: TF warning before detection

This can happen before the robot reaches the new-wheel detection pose. The full sequence script starts a joint-state stream at the detection pose, so the warning should disappear during the new-wheel detection stage.

---

## Notes

- Do not run `demo_ompl_only.launch.py` for the full simulation because it may start an extra `robot_state_publisher`.
- Use `demo_ompl_only_movegroup_only.launch.py`.
- The camera bridge is required for the OpenCV detector.
- The full task is executed by `scripts/run_old_dump_then_auto_new_grab.sh`.


## Contributors

This project was developed with contributions from:

- Fahad Mannan
- Seth Diaz
- Gifty Quayson
- An Nguyen

## License

This project is released under the MIT License. See the `LICENSE` file for details.
