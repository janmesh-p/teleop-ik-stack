#!/usr/bin/env python3
"""Operator target from an ArUco marker seen by the RealSense color camera.

Publishes the marker's 6-DoF pose in the camera optical frame, stamped with
the image capture time (host clock, synced across machines by chrony), plus
per-frame timing so capture-to-detection latency can be measured.

  /camera/camera/color/image_raw     sensor_msgs/Image (rgb8)
  /camera/camera/color/camera_info   sensor_msgs/CameraInfo (intrinsics)
  -> /operator/marker_pose           geometry_msgs/PoseStamped
  -> /operator/marker_timing         std_msgs/String (JSON per frame)

    python3 aruco_pose_node.py --size 0.0794 --id 0
"""

import argparse
import json
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def rvec_to_quat(rvec):
    R, _ = cv2.Rodrigues(rvec)
    t = np.trace(R)
    if t > 0:
        s = 2.0 * np.sqrt(t + 1.0)
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    w = (R[k, j] - R[j, k]) / s
    return q[0], q[1], q[2], w


class ArucoPose(Node):
    def __init__(self, args):
        super().__init__("aruco_pose")
        self.size, self.marker_id = args.size, args.id
        self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, "DetectorParameters_create") \
            else cv2.aruco.DetectorParameters()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.K = self.D = None
        self.frame_id = "camera_color_optical_frame"
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.on_image, qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, "/operator/marker_pose", 10)
        self.timing_pub = self.create_publisher(String, "/operator/marker_timing", 10)
        self.frames = self.detections = 0
        self.lat_capture_to_rx = deque(maxlen=2000)
        self.lat_capture_to_pose = deque(maxlen=2000)
        self.proc = deque(maxlen=2000)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f"waiting for camera_info; marker id {self.marker_id}, size {self.size * 1000:.1f} mm")

    def on_info(self, m):
        if self.K is None:
            self.K = np.array(m.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(m.d, dtype=np.float64)
            self.frame_id = m.header.frame_id or self.frame_id
            self.get_logger().info(f"intrinsics: fx {self.K[0, 0]:.1f} fy {self.K[1, 1]:.1f} "
                                   f"cx {self.K[0, 2]:.1f} cy {self.K[1, 2]:.1f}, frame {self.frame_id}")

    def on_image(self, m):
        t_rx = time.time()
        if self.K is None:
            return
        self.frames += 1
        t_cap = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dict, parameters=self.params)
        found = ids is not None and self.marker_id in ids.flatten()
        rec = {"seq": self.frames, "t_capture": t_cap, "t_rx": t_rx, "found": bool(found)}
        if found:
            idx = int(np.where(ids.flatten() == self.marker_id)[0][0])
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers([corners[idx]], self.size, self.K, self.D)
            rvec, tvec = rvecs[0][0], tvecs[0][0]
            t_pose = time.time()
            p = PoseStamped()
            p.header.stamp = m.header.stamp
            p.header.frame_id = self.frame_id
            p.pose.position.x, p.pose.position.y, p.pose.position.z = map(float, tvec)
            (p.pose.orientation.x, p.pose.orientation.y,
             p.pose.orientation.z, p.pose.orientation.w) = map(float, rvec_to_quat(rvec))
            self.pose_pub.publish(p)
            self.detections += 1
            self.lat_capture_to_pose.append((t_pose - t_cap) * 1000)
            rec.update({"t_pose": t_pose, "xyz": [round(float(v), 4) for v in tvec]})
        t_done = time.time()
        self.lat_capture_to_rx.append((t_rx - t_cap) * 1000)
        self.proc.append((t_done - t_rx) * 1000)
        self.timing_pub.publish(String(data=json.dumps(rec)))

    def report(self):
        if not self.frames:
            self.get_logger().info("no frames yet")
            return

        def pct(d):
            a = np.array(d)
            return "n/a" if a.size == 0 else f"p50 {np.percentile(a, 50):.1f} p95 {np.percentile(a, 95):.1f} p99 {np.percentile(a, 99):.1f} ms"

        self.get_logger().info(
            f"frames {self.frames}, detected {100 * self.detections / self.frames:.0f}% | "
            f"capture->receive {pct(self.lat_capture_to_rx)} | processing {pct(self.proc)} | "
            f"capture->pose {pct(self.lat_capture_to_pose)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", type=float, default=0.0794, help="black square edge, metres")
    ap.add_argument("--id", type=int, default=0)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = ArucoPose(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
