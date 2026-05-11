#!/usr/bin/env bash

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"

# ============================================================
# OLD WHEEL REMOVAL + DUMP + AUTONOMOUS NEW WHEEL GRAB
#
# Requires these already running:
# 1) Gazebo wheel-changing simulation
# 2) MoveIt:
#    ros2 launch wheel_changing_moveit_config demo_ompl_only.launch.py
# 3) Wheel detector:
#    ros2 run wheel_changing_vision_ik wheel_3d_detector --ros-args \
#      -p camera_frame:=ee_camera_optical_frame \
#      -p dark_v_max:=120 \
#      -p min_area:=300.0
# ============================================================


S=0.30
B=0.50

p()  { sleep "$S"; }
pb() { sleep "$B"; }


start_detection_joint_state_stream () {
  echo "Starting continuous ROS /joint_states stream for new-wheel detection TF..."

  python3 - <<'PYJS' >/tmp/new_wheel_detection_joint_state_pub.log 2>&1 &
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class DetectionPoseJointStatePub(Node):
    def __init__(self):
        super().__init__("detection_pose_joint_state_pub")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(0.05, self.publish_js)

        self.names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
            "ee_outer_hand_slide_joint",
            "ee_outer_hand1_joint",
            "ee_outer_hand2_joint",
            "ee_outer_hand3_joint",
            "ee_socket_slide_joint",
            "ee_socket_spin_joint",
            "ee_finger1_joint",
            "ee_finger2_joint",
            "ee_finger3_joint",
        ]

        # New-wheel detection pose
        self.positions = [
            4.4,
            -1.4,
            -1.8,
            -1.3,
            1.5708,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]

    def publish_js(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = self.positions
        self.pub.publish(msg)

rclpy.init()
node = DetectionPoseJointStatePub()
rclpy.spin(node)
PYJS

  DETECTION_JS_PID=$!
  sleep 1.0
}

stop_detection_joint_state_stream () {
  if [ -n "${DETECTION_JS_PID:-}" ]; then
    echo "Stopping detection-pose /joint_states stream..."
    kill "$DETECTION_JS_PID" >/dev/null 2>&1 || true
  fi
}

cmd_all_socket_fingers () {
  value="$1"

  gz topic -t /ur10e/ee_finger1_joint/cmd_pos -m gz.msgs.Double -p "data: ${value}" &
  p1=$!
  gz topic -t /ur10e/ee_finger2_joint/cmd_pos -m gz.msgs.Double -p "data: ${value}" &
  p2=$!
  gz topic -t /ur10e/ee_finger3_joint/cmd_pos -m gz.msgs.Double -p "data: ${value}" &
  p3=$!

  wait $p1 $p2 $p3
}


publish_ros_joint_state_detection_pose () {
  # Publish ROS /joint_states matching the Gazebo new-wheel detection pose.
  # This keeps robot_state_publisher TF consistent with the real Gazebo camera pose.
  ros2 topic pub --times 5 --rate 10 /joint_states sensor_msgs/msg/JointState "{
    name: [
      'shoulder_pan_joint',
      'shoulder_lift_joint',
      'elbow_joint',
      'wrist_1_joint',
      'wrist_2_joint',
      'wrist_3_joint'
    ],
    position: [
      4.4,
      -1.4,
      -1.8,
      -1.3,
      1.5708,
      0.0
    ]
  }" >/dev/null 2>&1 || true
}

cmd_all_outer_hands () {
  value="$1"

  gz topic -t /ur10e/ee_outer_hand1_joint/cmd_pos -m gz.msgs.Double -p "data: ${value}" &
  p1=$!
  gz topic -t /ur10e/ee_outer_hand2_joint/cmd_pos -m gz.msgs.Double -p "data: ${value}" &
  p2=$!
  gz topic -t /ur10e/ee_outer_hand3_joint/cmd_pos -m gz.msgs.Double -p "data: ${value}" &
  p3=$!

  wait $p1 $p2 $p3
}

echo "Checking joint topics..."
gz topic -l | grep -Ei "wheel_stand|new_wheel|installed_new_wheel|lug_nut_wheel|detach|attach" || true

# Keep optional joints detached at startup if present
gz topic -t /installed_new_wheel_joint/detach -m gz.msgs.Empty -p "" || true
p
gz topic -t /lug_nut_wheel_joint/detach -m gz.msgs.Empty -p "" || true
p


# ============================================================
# 1) Move to old wheel stand approach pose
# Updated old-wheel shoulder_pan_joint = 0.0
# ============================================================

echo "Moving to old wheel approach pose..."

gz topic -t /ur10e/shoulder_pan_joint/cmd_pos -m gz.msgs.Double -p "data: 0.0"
pb

gz topic -t /ur10e/elbow_joint/cmd_pos -m gz.msgs.Double -p "data: -1.8"
pb

gz topic -t /ur10e/shoulder_lift_joint/cmd_pos -m gz.msgs.Double -p "data: -1.2"
pb

gz topic -t /ur10e/wrist_1_joint/cmd_pos -m gz.msgs.Double -p "data: -0.04"
pb

gz topic -t /ur10e/wrist_2_joint/cmd_pos -m gz.msgs.Double -p "data: 1.5754"
pb

gz topic -t /ur10e/wrist_3_joint/cmd_pos -m gz.msgs.Double -p "data: 0.0"
pb


# ============================================================
# 2) Reset tool joints
# ============================================================

echo "Resetting tool joints..."

cmd_all_outer_hands 0.000
p

gz topic -t /ur10e/ee_outer_hand_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.000"
p

cmd_all_socket_fingers 0.020
p

gz topic -t /ur10e/ee_socket_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.000"
p

gz topic -t /ur10e/ee_socket_spin_joint/cmd_pos -m gz.msgs.Double -p "data: 0.0"
p


# ============================================================
# 3) Remove lug nut with socket
# ============================================================

echo "Removing lug nut..."

gz topic -t /ur10e/ee_socket_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.1355"
p

cmd_all_socket_fingers 0.015
p
cmd_all_socket_fingers 0.010
p
cmd_all_socket_fingers 0.005
p
cmd_all_socket_fingers 0.000
p

for v in 0.75 1.50 2.25 3.00 3.75 4.50 5.25 6.28; do
  gz topic -t /ur10e/ee_socket_spin_joint/cmd_pos -m gz.msgs.Double -p "data: ${v}"
  p
done

# Old socket retract values
gz topic -t /ur10e/ee_socket_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.120"
p

gz topic -t /ur10e/ee_socket_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.090"
p

gz topic -t /ur10e/ee_socket_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.050"
p


# ============================================================
# 4) Grab old wheel
# ============================================================

echo "Grabbing old wheel..."

cmd_all_outer_hands 0.000
p

for v in 0.080 0.140 0.200; do
  gz topic -t /ur10e/ee_outer_hand_slide_joint/cmd_pos -m gz.msgs.Double -p "data: ${v}"
  p
done

for v in 0.020 0.035 0.050 0.060; do
  cmd_all_outer_hands "${v}"
  p
done


# ============================================================
# 5) Detach and pull old wheel off stand
# ============================================================

echo "Detaching old wheel from stand..."

gz topic -t /wheel_stand_joint/detach -m gz.msgs.Empty -p ""
p

cmd_all_outer_hands 0.060
p

# Do not fully retract; keep wheel held securely
gz topic -t /ur10e/ee_outer_hand_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.135"
p


# ============================================================
# 6) Move to dumping zone and release old wheel
# Saved dump pose: pan=2.2, lift=-1.9, elbow=-1.3, wrist1=-1.3
# ============================================================

echo "Moving old wheel to dumping zone..."

cmd_all_outer_hands 0.060
p

for v in 0.8 1.4 2.2; do
  gz topic -t /ur10e/shoulder_pan_joint/cmd_pos -m gz.msgs.Double -p "data: ${v}"
  pb
done

gz topic -t /ur10e/shoulder_lift_joint/cmd_pos -m gz.msgs.Double -p "data: -1.5"
pb

gz topic -t /ur10e/elbow_joint/cmd_pos -m gz.msgs.Double -p "data: -1.5"
pb

gz topic -t /ur10e/wrist_1_joint/cmd_pos -m gz.msgs.Double -p "data: -0.8"
pb

gz topic -t /ur10e/shoulder_lift_joint/cmd_pos -m gz.msgs.Double -p "data: -1.9"
pb

gz topic -t /ur10e/elbow_joint/cmd_pos -m gz.msgs.Double -p "data: -1.3"
pb

gz topic -t /ur10e/wrist_1_joint/cmd_pos -m gz.msgs.Double -p "data: -1.3"
pb

echo "Releasing old wheel..."

# ============================================================
# 9) Release old wheel at dumping zone
# Exact sequence from uploaded file
# ============================================================

cmd_all_outer_hands 0.040
p

cmd_all_outer_hands 0.020
p

cmd_all_outer_hands 0.000
p


# ============================================================
# 10) Retract hands after releasing old wheel
# Exact values from uploaded file
# ============================================================

gz topic -t /ur10e/ee_outer_hand_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.080"
gz topic -t /ur10e/ee_outer_hand_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.000"


# ============================================================
# 11) Move to new wheel zone / OpenCV detection pose
# Exact arm sequence from uploaded file
# No socket spin command here
# ============================================================

gz topic -t /ur10e/shoulder_lift_joint/cmd_pos -m gz.msgs.Double -p "data: -1.4"
gz topic -t /ur10e/elbow_joint/cmd_pos -m gz.msgs.Double -p "data: -1.8"
gz topic -t /ur10e/wrist_1_joint/cmd_pos -m gz.msgs.Double -p "data: -1.3"
gz topic -t /ur10e/wrist_2_joint/cmd_pos -m gz.msgs.Double -p "data: 1.5708"
gz topic -t /ur10e/wrist_3_joint/cmd_pos -m gz.msgs.Double -p "data: 0"
gz topic -t /ur10e/shoulder_pan_joint/cmd_pos -m gz.msgs.Double -p "data: 4.4"


# ============================================================
# 11.5) Correct TF for OpenCV/MoveIt detection
# This is only for perception; arm values above remain from uploaded file.
# ============================================================

start_detection_joint_state_stream
sleep 1.0


# ============================================================
# 12) Move to new wheel grabbing pose
# HYBRID PART:
# Use OpenCV + MoveIt IK with recalibrated offsets instead of manual pose.
# ============================================================

ros2 run wheel_changing_vision_ik auto_move_to_wheel_grab --ros-args \
  -p execute:=true \
  -p x_offset:=-0.013273 \
  -p y_offset:=-0.066320 \
  -p z_offset:=0.413476 \
  -p steps:=10 \
  -p step_sleep:=0.01

stop_detection_joint_state_stream


# ============================================================
# 13) Extend hands around new wheel
# Exact values from uploaded file
# ============================================================

cmd_all_outer_hands 0.000
p

gz topic -t /ur10e/ee_outer_hand_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.080"
p

gz topic -t /ur10e/ee_outer_hand_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.140"
p

gz topic -t /ur10e/ee_outer_hand_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.200"
p


# ============================================================
# 14) Grip new wheel tightly
# Exact values from uploaded file
# ============================================================

cmd_all_outer_hands 0.020
p

cmd_all_outer_hands 0.035
p

cmd_all_outer_hands 0.050
p

cmd_all_outer_hands 0.060
p


# ============================================================
# 15) Detach new wheel from blue supply zone AFTER strong grip
# ============================================================

gz topic -t /new_wheel_joint/detach -m gz.msgs.Empty -p ""
p


# ============================================================
# 16) Slightly pick/lift new wheel while gripping
# Exact values from uploaded file
# ============================================================

cmd_all_outer_hands 0.060
p

gz topic -t /ur10e/ee_outer_hand_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.100"
p

gz topic -t /ur10e/elbow_joint/cmd_pos -m gz.msgs.Double -p "data: -1.40"
p

gz topic -t /ur10e/shoulder_lift_joint/cmd_pos -m gz.msgs.Double -p "data: -1.0"
p

gz topic -t /ur10e/elbow_joint/cmd_pos -m gz.msgs.Double -p "data: -1.45"
p


# ============================================================
# 17) Carry new wheel back toward changing stand
# Exact values from uploaded file
# ============================================================

cmd_all_outer_hands 0.060
p

gz topic -t /ur10e/shoulder_pan_joint/cmd_pos -m gz.msgs.Double -p "data: 4.8"
pb

gz topic -t /ur10e/shoulder_pan_joint/cmd_pos -m gz.msgs.Double -p "data: 5.2"
pb

gz topic -t /ur10e/shoulder_pan_joint/cmd_pos -m gz.msgs.Double -p "data: 5.6"
pb

gz topic -t /ur10e/shoulder_pan_joint/cmd_pos -m gz.msgs.Double -p "data: 6.0"
pb

gz topic -t /ur10e/shoulder_pan_joint/cmd_pos -m gz.msgs.Double -p "data: 6.28319"
pb


# ============================================================
# 18) Place new wheel at stand position
# Exact values from uploaded file
# ============================================================

gz topic -t /ur10e/elbow_joint/cmd_pos -m gz.msgs.Double -p "data: -1.8"
pb

gz topic -t /ur10e/shoulder_lift_joint/cmd_pos -m gz.msgs.Double -p "data: -1.2"
pb

gz topic -t /ur10e/wrist_1_joint/cmd_pos -m gz.msgs.Double -p "data: -0.04"
pb

gz topic -t /ur10e/wrist_2_joint/cmd_pos -m gz.msgs.Double -p "data: 1.5754"
pb

gz topic -t /ur10e/wrist_3_joint/cmd_pos -m gz.msgs.Double -p "data: 0.0"
pb

gz topic -t /ur10e/ee_outer_hand_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.175"
p


# ============================================================
# 19) Attach installed new wheel to stand
# ============================================================

gz topic -t /installed_new_wheel_joint/attach -m gz.msgs.Empty -p ""
p


# ============================================================
# 20) Put lug nut back on new wheel
# Exact values from uploaded file
# ============================================================

cmd_all_socket_fingers 0.000
p

gz topic -t /ur10e/ee_socket_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.090"
p

gz topic -t /ur10e/ee_socket_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.120"
p

gz topic -t /ur10e/ee_socket_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.1355"
p


# ============================================================
# 21) Reverse spin to seat/tighten lug nut
# Exact values from uploaded file
# ============================================================

gz topic -t /ur10e/ee_socket_spin_joint/cmd_pos -m gz.msgs.Double -p "data: -0.25"
p
gz topic -t /ur10e/ee_socket_spin_joint/cmd_pos -m gz.msgs.Double -p "data: -0.50"
p
gz topic -t /ur10e/ee_socket_spin_joint/cmd_pos -m gz.msgs.Double -p "data: -0.75"
p
gz topic -t /ur10e/ee_socket_spin_joint/cmd_pos -m gz.msgs.Double -p "data: -1.50"
p
gz topic -t /ur10e/ee_socket_spin_joint/cmd_pos -m gz.msgs.Double -p "data: -2.25"
p
gz topic -t /ur10e/ee_socket_spin_joint/cmd_pos -m gz.msgs.Double -p "data: -3.00"
p
gz topic -t /ur10e/ee_socket_spin_joint/cmd_pos -m gz.msgs.Double -p "data: -3.75"
p
gz topic -t /ur10e/ee_socket_spin_joint/cmd_pos -m gz.msgs.Double -p "data: -4.50"
p
gz topic -t /ur10e/ee_socket_spin_joint/cmd_pos -m gz.msgs.Double -p "data: -5.25"
p
gz topic -t /ur10e/ee_socket_spin_joint/cmd_pos -m gz.msgs.Double -p "data: -6.28"
p


# ============================================================
# 21.5) Activate lug-nut-to-wheel joint
# Must happen after nut is placed/spun, before socket fingers release
# ============================================================

gz topic -t /lug_nut_wheel_joint/attach -m gz.msgs.Empty -p ""
sleep 0.3


# ============================================================
# 22) Release lug nut from socket fingers
# Exact values from uploaded file
# ============================================================

cmd_all_socket_fingers 0.005
p

cmd_all_socket_fingers 0.010
p

cmd_all_socket_fingers 0.015
p

cmd_all_socket_fingers 0.020
p


# ============================================================
# 23) Retract socket after nut placement
# Exact values from uploaded file
# ============================================================

gz topic -t /ur10e/ee_socket_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.090"
p

gz topic -t /ur10e/ee_socket_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.050"
p

gz topic -t /ur10e/ee_socket_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.000"
p


# ============================================================
# 24) Release outer hands from installed new wheel
# Exact values from uploaded file
# ============================================================

cmd_all_outer_hands 0.040
p

cmd_all_outer_hands 0.020
p

cmd_all_outer_hands 0.000
p

gz topic -t /ur10e/ee_outer_hand_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.140"
p

gz topic -t /ur10e/ee_outer_hand_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.080"
p

gz topic -t /ur10e/ee_outer_hand_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.000"
p


# ============================================================
# 25) Reset tool joints
# Exact values from uploaded file
# ============================================================

cmd_all_outer_hands 0.000
p

gz topic -t /ur10e/ee_outer_hand_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.000"
p

cmd_all_socket_fingers 0.00
p

gz topic -t /ur10e/ee_socket_slide_joint/cmd_pos -m gz.msgs.Double -p "data: 0.000"
p


# ============================================================
# 26) Return arm to confirmed initial/home position
# Exact values from uploaded file
# ============================================================

gz topic -t /ur10e/ee_socket_spin_joint/cmd_pos -m gz.msgs.Double -p "data: 0"
pb

gz topic -t /ur10e/wrist_3_joint/cmd_pos -m gz.msgs.Double -p "data: 0"
pb

gz topic -t /ur10e/wrist_2_joint/cmd_pos -m gz.msgs.Double -p "data: 1.5708"
pb

gz topic -t /ur10e/wrist_1_joint/cmd_pos -m gz.msgs.Double -p "data: -0.4411"
pb

gz topic -t /ur10e/elbow_joint/cmd_pos -m gz.msgs.Double -p "data: -2.6320"
pb

gz topic -t /ur10e/shoulder_lift_joint/cmd_pos -m gz.msgs.Double -p "data: -0.0339"
pb

gz topic -t /ur10e/shoulder_pan_joint/cmd_pos -m gz.msgs.Double -p "data: 6.28319"
pb


# ============================================================
# COMPLETE
# ============================================================

echo "DONE: hybrid sequence complete. OpenCV/MoveIt used for new-wheel approach; all other movement values follow uploaded file."
