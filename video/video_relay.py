#!/usr/bin/env python3
"""Laptop side: JPEG-encode Isaac's camera frames for the operator link.

  /isaac_camera/image_raw (rgb8)  ->  /isaac_camera/jpeg (CompressedImage)

The JPEG message keeps the render stamp, so the receiver measures
render -> decoded on synced clocks. Encode time per frame goes on
/isaac_camera/jpeg_meta as JSON.

    source /opt/ros/jazzy/setup.bash
    python3 video_relay.py --quality 80
"""

import argparse
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


class Relay(Node):
    def __init__(self, quality):
        super().__init__("video_relay")
        self.params = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        self.pub = self.create_publisher(CompressedImage, "/isaac_camera/jpeg", qos_profile_sensor_data)
        self.meta = self.create_publisher(String, "/isaac_camera/jpeg_meta", 10)
        self.create_subscription(Image, "/isaac_camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.enc_ms = []
        self.create_timer(5.0, self.report)
        self.get_logger().info(f"relaying to /isaac_camera/jpeg at quality {quality}")

    def on_image(self, m):
        ch = len(m.data) // (m.width * m.height)
        img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, ch)
        t0 = time.perf_counter()
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(img[:, :, :3], cv2.COLOR_RGB2BGR), self.params)
        enc = (time.perf_counter() - t0) * 1000
        if not ok:
            return
        out = CompressedImage()
        out.header = m.header  # keep the render stamp
        out.format = "jpeg"
        out.data = buf.tobytes()
        self.pub.publish(out)
        self.enc_ms.append(enc)
        s = m.header.stamp
        self.meta.publish(String(data=json.dumps({"stamp": s.sec + s.nanosec * 1e-9, "encode_ms": round(enc, 3),
                                                  "raw_bytes": len(m.data), "jpeg_bytes": len(out.data)})))

    def report(self):
        if self.enc_ms:
            a = np.array(self.enc_ms)
            self.get_logger().info(f"frames {a.size} | encode p50 {np.percentile(a, 50):.2f} p99 {np.percentile(a, 99):.2f} ms")
            self.enc_ms.clear()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quality", type=int, default=80)
    a, _ = ap.parse_known_args()
    rclpy.init()
    n = Relay(a.quality)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
