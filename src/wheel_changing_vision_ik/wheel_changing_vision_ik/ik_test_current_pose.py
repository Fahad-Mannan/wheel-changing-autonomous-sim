#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from moveit_msgs.srv import GetPositionIK

import tf2_ros


class IKTestCurrentPose(Node):
    def __init__(self):
        super().__init__("ik_test_current_pose")

        self.base_frame = "base_link"
        self.tip_frame = "wrist_3_link"
        self.group_name = "ur_manipulator"

        self.latest_joint_state = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            10,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        self.timer = self.create_timer(1.0, self.run_once)
        self.already_ran = False

        self.get_logger().info("IK test node started.")
        self.get_logger().info("Waiting for /joint_states, TF, and /compute_ik...")

    def joint_state_cb(self, msg):
        self.latest_joint_state = msg

    def run_once(self):
        if self.already_ran:
            return

        if self.latest_joint_state is None:
            self.get_logger().info("Waiting for /joint_states...")
            return

        if not self.ik_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().info("Waiting for /compute_ik service...")
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tip_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
        except Exception as exc:
            self.get_logger().info(f"Waiting for TF {self.base_frame} -> {self.tip_frame}: {exc}")
            return

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z

        pose.pose.orientation = tf.transform.rotation

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state.joint_state = self.latest_joint_state
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 2
        req.ik_request.timeout.nanosec = 0

        self.get_logger().info("Calling /compute_ik for current wrist_3_link pose...")
        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_response_cb)

        self.already_ran = True

    def ik_response_cb(self, future):
        try:
            res = future.result()
        except Exception as exc:
            self.get_logger().error(f"IK service call failed: {exc}")
            return

        if res.error_code.val == res.error_code.SUCCESS:
            self.get_logger().info("IK SUCCESS.")

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

            self.get_logger().info("IK joint solution:")
            for joint in wanted:
                if joint in names:
                    i = names.index(joint)
                    self.get_logger().info(f"  {joint}: {positions[i]:.6f}")
        else:
            self.get_logger().error(f"IK FAILED. MoveIt error code: {res.error_code.val}")


def main(args=None):
    rclpy.init(args=args)
    node = IKTestCurrentPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
