#!/usr/bin/env python3

import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

import tf2_ros


class Wheel3DDetector(Node):
    def __init__(self):
        super().__init__("wheel_3d_detector")

        self.declare_parameter("rgb_topic", "/ee_rgbd_camera/image")
        self.declare_parameter("depth_topic", "/ee_rgbd_camera/depth_image")
        self.declare_parameter("camera_info_topic", "/ee_rgbd_camera/camera_info")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "ee_camera_optical_frame")
        self.declare_parameter("dark_v_max", 90)
        self.declare_parameter("min_area", 700.0)

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.dark_v_max = int(self.get_parameter("dark_v_max").value)
        self.min_area = float(self.get_parameter("min_area").value)

        self.bridge = CvBridge()

        self.latest_depth = None
        self.latest_depth_header = None
        self.camera_info = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, self.rgb_topic, self.rgb_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.info_cb, qos_profile_sensor_data)

        self.annotated_pub = self.create_publisher(Image, "/wheel_detector/annotated_image", 10)
        self.pixel_pub = self.create_publisher(PointStamped, "/wheel_detector/new_wheel_center_px", 10)
        self.camera_point_pub = self.create_publisher(PointStamped, "/wheel_detector/new_wheel_point_camera", 10)
        self.base_point_pub = self.create_publisher(PointStamped, "/wheel_detector/new_wheel_point_base", 10)

        self.frame_count = 0

        self.get_logger().info("Wheel 3D detector started.")
        self.get_logger().info(f"RGB:   {self.rgb_topic}")
        self.get_logger().info(f"Depth: {self.depth_topic}")
        self.get_logger().info(f"Info:  {self.camera_info_topic}")
        self.get_logger().info(f"Using camera TF frame override: {self.camera_frame}")

    def depth_cb(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth = np.array(depth)
            self.latest_depth_header = msg.header
        except Exception as exc:
            self.get_logger().warn(f"Depth conversion failed: {exc}")

    def info_cb(self, msg):
        self.camera_info = msg

    def rgb_cb(self, msg):
        if self.latest_depth is None or self.camera_info is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"RGB conversion failed: {exc}")
            return

        detection = self.detect_wheel(frame)

        annotated = frame.copy()

        if detection is None:
            cv2.putText(
                annotated,
                "Wheel not detected",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
            self.publish_annotated(annotated, msg.header)
            return

        u, v, radius, area, circularity = detection

        depth_m = self.get_depth_meters(u, v)
        if depth_m is None:
            cv2.putText(
                annotated,
                "Wheel detected, depth invalid",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
            self.publish_annotated(annotated, msg.header)
            return

        point_cam = self.pixel_to_camera_point(u, v, depth_m, self.camera_frame)

        point_base = self.transform_point_to_base(point_cam)

        self.publish_results(point_cam, point_base, u, v, radius, msg.header)

        cv2.circle(annotated, (int(u), int(v)), int(radius), (0, 255, 0), 2)
        cv2.circle(annotated, (int(u), int(v)), 4, (0, 0, 255), -1)

        label = f"wheel px=({int(u)},{int(v)}) depth={depth_m:.3f} m"
        cv2.putText(
            annotated,
            label,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        if point_base is not None:
            label2 = f"base=({point_base.point.x:.3f}, {point_base.point.y:.3f}, {point_base.point.z:.3f})"
            cv2.putText(
                annotated,
                label2,
                (20, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
            )

        self.publish_annotated(annotated, msg.header)

        self.frame_count += 1
        if self.frame_count % 20 == 0:
            self.get_logger().info(
                f"Wheel px=({u:.1f},{v:.1f}), radius={radius:.1f}, "
                f"depth={depth_m:.3f} m, area={area:.1f}, circularity={circularity:.2f}"
            )
            if point_base is not None:
                self.get_logger().info(
                    f"Wheel point in base_link: "
                    f"x={point_base.point.x:.3f}, "
                    f"y={point_base.point.y:.3f}, "
                    f"z={point_base.point.z:.3f}"
                )

    def detect_wheel(self, frame):
        """
        Robust detector for the simple wheel:
        1) detect gray circular hub
        2) use hub center as wheel center
        3) estimate tire radius from black pixels around that center

        This avoids the old problem where the black stand merged with the black tire.
        """
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ------------------------------------------------------------
        # 1) Detect gray hub: low saturation, medium brightness.
        # The hub is gray, while tire is black and fingers/nut are yellow.
        # ------------------------------------------------------------
        lower_gray = np.array([0, 0, 60], dtype=np.uint8)
        upper_gray = np.array([180, 70, 220], dtype=np.uint8)
        gray_mask = cv2.inRange(hsv, lower_gray, upper_gray)

        # Remove floor/background lower region. Wheel hub should be above floor.
        gray_mask[int(0.78 * h):h, :] = 0

        # Clean mask
        kernel = np.ones((5, 5), np.uint8)
        gray_mask = cv2.morphologyEx(gray_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        gray_mask = cv2.morphologyEx(gray_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(gray_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_score = -1.0

        for c in contours:
            area = cv2.contourArea(c)
            if area < 250:
                continue

            perimeter = cv2.arcLength(c, True)
            if perimeter <= 1e-6:
                continue

            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            x, y, bw, bh = cv2.boundingRect(c)
            aspect = bw / float(bh + 1e-6)

            if not (0.55 <= circularity <= 1.35):
                continue
            if not (0.55 <= aspect <= 1.65):
                continue

            M = cv2.moments(c)
            if abs(M["m00"]) < 1e-6:
                continue

            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]

            # Prefer objects near the image center area, not background patches.
            center_penalty = abs(cx - 0.5 * w) / w + abs(cy - 0.45 * h) / h
            score = area * circularity - 800.0 * center_penalty

            if score > best_score:
                best_score = score
                best = (float(cx), float(cy), float(area), float(circularity))

        if best is None:
            return None

        cx, cy, hub_area, hub_circularity = best

        # ------------------------------------------------------------
        # 2) Estimate tire radius from black tire pixels around hub center.
        # Remove the lower stand region from black mask.
        # ------------------------------------------------------------
        lower_dark = np.array([0, 0, 0], dtype=np.uint8)
        upper_dark = np.array([180, 255, self.dark_v_max], dtype=np.uint8)
        dark_mask = cv2.inRange(hsv, lower_dark, upper_dark)

        # Remove yellow fingers/nut if any overlap
        lower_yellow = np.array([15, 60, 60], dtype=np.uint8)
        upper_yellow = np.array([45, 255, 255], dtype=np.uint8)
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        dark_mask = cv2.bitwise_and(dark_mask, cv2.bitwise_not(yellow_mask))

        # Remove vertical stand below wheel center, because it is black too.
        x0 = max(0, int(cx - 35))
        x1 = min(w, int(cx + 35))
        y0 = min(h, int(cy + 25))
        dark_mask[y0:h, x0:x1] = 0

        ys, xs = np.where(dark_mask > 0)

        if xs.size > 20:
            # Use only black pixels near the hub center, not far background.
            dx = xs.astype(np.float32) - float(cx)
            dy = ys.astype(np.float32) - float(cy)
            dist = np.sqrt(dx * dx + dy * dy)

            # Tire radius should be outside hub but not the whole image.
            valid = dist[(dist > 25) & (dist < 220)]
            if valid.size > 20:
                radius = float(np.percentile(valid, 88))
            else:
                radius = 70.0
        else:
            radius = 70.0

        # Keep radius in reasonable range
        radius = max(35.0, min(radius, 160.0))

        # Return same format used by the rest of the node:
        # u, v, radius, area, circularity
        return (float(cx), float(cy), float(radius), float(hub_area), float(hub_circularity))

    def get_depth_meters(self, u, v):
        depth = self.latest_depth
        if depth is None:
            return None

        h, w = depth.shape[:2]
        u = int(round(u))
        v = int(round(v))

        if u < 0 or u >= w or v < 0 or v >= h:
            return None

        r = 4
        u0 = max(0, u - r)
        u1 = min(w, u + r + 1)
        v0 = max(0, v - r)
        v1 = min(h, v + r + 1)

        roi = depth[v0:v1, u0:u1].astype(np.float32).flatten()
        roi = roi[np.isfinite(roi)]
        roi = roi[roi > 0.0]

        if roi.size == 0:
            return None

        z = float(np.median(roi))

        # If depth comes in millimeters as uint16, convert to meters.
        if z > 20.0:
            z = z / 1000.0

        if z <= 0.0 or not math.isfinite(z):
            return None

        return z

    def pixel_to_camera_point(self, u, v, z, frame_id):
        k = self.camera_info.k
        fx = k[0]
        fy = k[4]
        cx = k[2]
        cy = k[5]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        p = PointStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = frame_id
        p.point.x = float(x)
        p.point.y = float(y)
        p.point.z = float(z)
        return p

    def transform_point_to_base(self, point_cam):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                point_cam.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.3),
            )

            # Manual transform point using quaternion.
            q = tf.transform.rotation
            t = tf.transform.translation

            x, y, z = point_cam.point.x, point_cam.point.y, point_cam.point.z
            qx, qy, qz, qw = q.x, q.y, q.z, q.w

            # Quaternion rotation: p' = q * p * q^-1
            # Expanded formula.
            ix =  qw * x + qy * z - qz * y
            iy =  qw * y + qz * x - qx * z
            iz =  qw * z + qx * y - qy * x
            iw = -qx * x - qy * y - qz * z

            rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
            ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
            rz = iz * qw + iw * -qz + ix * -qy - iy * -qx

            out = PointStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.base_frame
            out.point.x = rx + t.x
            out.point.y = ry + t.y
            out.point.z = rz + t.z

            return out

        except Exception as exc:
            self.get_logger().warn(f"TF transform to {self.base_frame} failed: {exc}")
            return None

    def publish_results(self, point_cam, point_base, u, v, radius, header):
        px = PointStamped()
        px.header = header
        px.point.x = float(u)
        px.point.y = float(v)
        px.point.z = float(radius)
        self.pixel_pub.publish(px)

        self.camera_point_pub.publish(point_cam)

        if point_base is not None:
            self.base_point_pub.publish(point_base)

    def publish_annotated(self, frame, header):
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header = header
            self.annotated_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"Annotated image publish failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = Wheel3DDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
