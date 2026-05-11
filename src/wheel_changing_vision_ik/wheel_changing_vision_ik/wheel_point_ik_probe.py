#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK

import tf2_ros


class WheelPointIKProbe(Node):
    def __init__(self):
        super().__init__("wheel_point_ik_probe")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tip_frame", "wrist_3_link")
        self.declare_parameter("group_name", "ur_manipulator")

        self.declare_parameter("x_offset", 0.0)
        self.declare_parameter("y_offset", 0.0)
        self.declare_parameter("z_offset", 0.0)
        self.declare_parameter("avoid_collisions", False)

        self.base_frame = self.get_parameter("base_frame").value
        self.tip_frame = self.get_parameter("tip_frame").value
        self.group_name = self.get_parameter("group_name").value

        self.x_offset = float(self.get_parameter("x_offset").value)
        self.y_offset = float(self.get_parameter("y_offset").value)
        self.z_offset = float(self.get_parameter("z_offset").value)
        self.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)

        self.latest_point = None
        self.latest_joint_state = None
        self.already_ran = False

        self.create_subscription(
            PointStamped,
            "/wheel_detector/new_wheel_point_base",
            self.point_cb,
            10,
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            10,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        self.timer = self.create_timer(0.5, self.try_ik)

        self.get_logger().info("Wheel-point IK probe started.")
        self.get_logger().info("Waiting for /wheel_detector/new_wheel_point_base, /joint_states, and /compute_ik...")
        self.get_logger().info(
            f"Offsets: x={self.x_offset:.3f}, y={self.y_offset:.3f}, z={self.z_offset:.3f}"
        )

    def point_cb(self, msg):
        self.latest_point = msg

    def joint_state_cb(self, msg):
        self.latest_joint_state = msg

    def try_ik(self):
        if self.already_ran:
            return

        if self.latest_point is None:
            self.get_logger().info("Waiting for detected wheel point...")
            return

        if self.latest_joint_state is None:
            self.get_logger().info("Waiting for /joint_states...")
            return

        if not self.ik_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().info("Waiting for /compute_ik...")
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tip_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
        except Exception as exc:
            self.get_logger().warn(f"Could not get current {self.tip_frame} orientation: {exc}")
            return

        target = PoseStamped()
        target.header.frame_id = self.base_frame
        target.header.stamp = self.get_clock().now().to_msg()

        target.pose.position.x = self.latest_point.point.x + self.x_offset
        target.pose.position.y = self.latest_point.point.y + self.y_offset
        target.pose.position.z = self.latest_point.point.z + self.z_offset

        # For the first test, keep the current wrist_3_link orientation.
        target.pose.orientation = tf.transform.rotation

        self.get_logger().info("Detected wheel point:")
        self.get_logger().info(
            f"  wheel base point = ({self.latest_point.point.x:.4f}, "
            f"{self.latest_point.point.y:.4f}, {self.latest_point.point.z:.4f})"
        )
        self.get_logger().info("Trying IK target:")
        self.get_logger().info(
            f"  target wrist_3_link = ({target.pose.position.x:.4f}, "
            f"{target.pose.position.y:.4f}, {target.pose.position.z:.4f})"
        )

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.tip_frame
        req.ik_request.robot_state.joint_state = self.latest_joint_state
        req.ik_request.pose_stamped = target
        req.ik_request.avoid_collisions = self.avoid_collisions
        req.ik_request.timeout.sec = 2
        req.ik_request.timeout.nanosec = 0

        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_response_cb)

        self.already_ran = True

    def ik_response_cb(self, future):
        try:
            res = future.result()
        except Exception as exc:
            self.get_logger().error(f"IK call failed: {exc}")
            return

        if res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error(f"IK FAILED. MoveIt error code: {res.error_code.val}")
            self.get_logger().error("Try changing x_offset, y_offset, or z_offset.")
            return

        self.get_logger().info("IK SUCCESS. Joint solution:")

        names = res.solution.joint_state.name
        positions = res.solution.joint_state.position

        wanted = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        for joint in wanted:
            if joint in names:
                i = names.index(joint)
                self.get_logger().info(f"  {joint}: {positions[i]:.6f}")

        self.get_logger().info("This node only computed IK. It did not move the robot.")


def main(args=None):
    rclpy.init(args=args)
    node = WheelPointIKProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
