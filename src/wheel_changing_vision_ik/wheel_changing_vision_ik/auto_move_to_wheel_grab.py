#!/usr/bin/env python3

import subprocess
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import JointState

from moveit_msgs.srv import GetPositionIK, GetPositionFK


class AutoMoveToWheelGrab(Node):
    def __init__(self):
        super().__init__("auto_move_to_wheel_grab")

        self.declare_parameter("execute", False)
        self.declare_parameter("steps", 25)
        self.declare_parameter("step_sleep", 0.06)

        # Calibrated from your detected wheel point and known new_wheel_grab pose
        self.declare_parameter("x_offset", -0.013273)
        self.declare_parameter("y_offset", -0.066320)
        self.declare_parameter("z_offset", 0.413476)

        self.declare_parameter("avoid_collisions", False)

        self.execute = bool(self.get_parameter("execute").value)
        self.steps = int(self.get_parameter("steps").value)
        self.step_sleep = float(self.get_parameter("step_sleep").value)

        self.x_offset = float(self.get_parameter("x_offset").value)
        self.y_offset = float(self.get_parameter("y_offset").value)
        self.z_offset = float(self.get_parameter("z_offset").value)

        self.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)

        self.group_name = "ur_manipulator"
        self.base_frame = "base_link"
        self.tip_frame = "wrist_3_link"

        self.arm_joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Known pose before moving toward new wheel grab
        self.new_wheel_detection_pose = {
            "shoulder_pan_joint": 4.4,
            "shoulder_lift_joint": -1.4,
            "elbow_joint": -1.8,
            "wrist_1_joint": -1.3,
            "wrist_2_joint": 1.5708,
            "wrist_3_joint": 0.0,
        }

        # Known good manual new wheel grab pose.
        # We use this as FK reference orientation and IK seed.
        self.new_wheel_grab_pose = {
            "shoulder_pan_joint": 4.4,
            "shoulder_lift_joint": -1.946,
            "elbow_joint": -1.3,
            "wrist_1_joint": -1.3,
            "wrist_2_joint": 1.5708,
            "wrist_3_joint": 0.0,
        }

        self.latest_point = None
        self.started = False

        self.create_subscription(
            PointStamped,
            "/wheel_detector/new_wheel_point_base",
            self.point_cb,
            10,
        )

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")

        self.timer = self.create_timer(0.5, self.try_start)

        self.get_logger().info("Auto move-to-wheel-grab node started.")
        self.get_logger().info(f"execute = {self.execute}")
        self.get_logger().info(
            f"offsets: x={self.x_offset:.6f}, y={self.y_offset:.6f}, z={self.z_offset:.6f}"
        )

    def point_cb(self, msg):
        self.latest_point = msg

    def try_start(self):
        if self.started:
            return

        if self.latest_point is None:
            self.get_logger().info("Waiting for /wheel_detector/new_wheel_point_base...")
            return

        if not self.fk_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().info("Waiting for /compute_fk...")
            return

        if not self.ik_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().info("Waiting for /compute_ik...")
            return

        self.started = True
        self.call_fk_for_reference_orientation()

    def make_joint_state_from_pose(self, pose_dict):
        js = JointState()
        js.name = list(self.arm_joints)
        js.position = [float(pose_dict[j]) for j in self.arm_joints]
        return js

    def call_fk_for_reference_orientation(self):
        req = GetPositionFK.Request()
        req.header.frame_id = self.base_frame
        req.fk_link_names = [self.tip_frame]
        req.robot_state.joint_state = self.make_joint_state_from_pose(self.new_wheel_grab_pose)

        self.get_logger().info("Calling /compute_fk for reference new_wheel_grab orientation...")
        future = self.fk_client.call_async(req)
        future.add_done_callback(self.fk_done_cb)

    def fk_done_cb(self, future):
        try:
            res = future.result()
        except Exception as exc:
            self.get_logger().error(f"FK call failed: {exc}")
            return

        if len(res.pose_stamped) < 1:
            self.get_logger().error("FK failed: no pose returned.")
            return

        reference_orientation = res.pose_stamped[0].pose.orientation

        target = PoseStamped()
        target.header.frame_id = self.base_frame
        target.header.stamp = self.get_clock().now().to_msg()

        target.pose.position.x = self.latest_point.point.x + self.x_offset
        target.pose.position.y = self.latest_point.point.y + self.y_offset
        target.pose.position.z = self.latest_point.point.z + self.z_offset
        target.pose.orientation = reference_orientation

        self.get_logger().info("Detected wheel point:")
        self.get_logger().info(
            f"  wheel = ({self.latest_point.point.x:.6f}, "
            f"{self.latest_point.point.y:.6f}, {self.latest_point.point.z:.6f})"
        )

        self.get_logger().info("IK target wrist_3_link:")
        self.get_logger().info(
            f"  target = ({target.pose.position.x:.6f}, "
            f"{target.pose.position.y:.6f}, {target.pose.position.z:.6f})"
        )

        self.call_ik(target)

    def call_ik(self, target):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.tip_frame
        req.ik_request.robot_state.joint_state = self.make_joint_state_from_pose(
            self.new_wheel_grab_pose
        )
        req.ik_request.pose_stamped = target
        req.ik_request.avoid_collisions = self.avoid_collisions
        req.ik_request.timeout.sec = 2
        req.ik_request.timeout.nanosec = 0

        self.get_logger().info("Calling /compute_ik...")
        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_done_cb)

    def ik_done_cb(self, future):
        try:
            res = future.result()
        except Exception as exc:
            self.get_logger().error(f"IK call failed: {exc}")
            return

        if res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error(f"IK FAILED. MoveIt error code: {res.error_code.val}")
            return

        names = res.solution.joint_state.name
        positions = res.solution.joint_state.position

        solution = {}

        self.get_logger().info("IK SUCCESS. Solution:")
        for joint in self.arm_joints:
            if joint not in names:
                self.get_logger().error(f"Joint missing from IK solution: {joint}")
                return
            i = names.index(joint)
            solution[joint] = float(positions[i])
            self.get_logger().info(f"  {joint}: {solution[joint]:.6f}")

        if not self.execute:
            self.get_logger().info("Dry run only. Robot was NOT moved.")
            self.get_logger().info("Run again with -p execute:=true to command Gazebo.")
            rclpy.shutdown()
            return

        self.get_logger().info("Executing smooth Gazebo joint commands...")
        self.execute_gazebo_motion(solution)
        self.get_logger().info("Finished moving arm to detected new-wheel grabbing pose.")
        rclpy.shutdown()

    def execute_gazebo_motion(self, target_solution):
        start = self.new_wheel_detection_pose

        for step in range(1, self.steps + 1):
            alpha = step / float(self.steps)

            cmds = {}
            for joint in self.arm_joints:
                q0 = float(start[joint])
                q1 = float(target_solution[joint])
                q = q0 + alpha * (q1 - q0)
                cmds[joint] = q

            self.publish_gz_joint_commands(cmds)
            time.sleep(self.step_sleep)

    def publish_gz_joint_commands(self, cmds):
        processes = []

        for joint, value in cmds.items():
            topic = f"/ur10e/{joint}/cmd_pos"
            cmd = [
                "gz",
                "topic",
                "-t",
                topic,
                "-m",
                "gz.msgs.Double",
                "-p",
                f"data: {value:.9f}",
            ]

            try:
                p = subprocess.Popen(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                processes.append(p)
            except Exception as exc:
                self.get_logger().error(f"Failed to publish {joint}: {exc}")

        for p in processes:
            p.wait()


def main(args=None):
    rclpy.init(args=args)
    node = AutoMoveToWheelGrab()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
