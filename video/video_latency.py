#!/usr/bin/env python3
"""Jetson side: operator video timing, raw vs JPEG.

Per frame, on chrony-synced clocks:
  raw:   render -> received                     (/isaac_camera/image_raw)
  jpeg:  render -> received -> decoded          (/isaac_camera/jpeg)
         plus the laptop's encode time from /isaac_camera/jpeg_meta
Also frame rate, gaps and bandwidth for each stream. Runs for --seconds,
then prints and saves a JSON summary. --show displays the decoded stream.

Render time here is the Isaac frame stamp, so this is render-to-display
latency for the simulated camera, not camera-capture-to-display.

    python3 video_latency.py --seconds 30 --out /ws/results/video.json
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


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Measure(Node):
    def __init__(self, show):
        super().__init__("video_latency")
        self.show = show
        self.raw = {"lat": [], "rx": [], "bytes": 0}
        self.jpg = {"lat": [], "rx": [], "bytes": 0, "decode": [], "total": []}
        self.enc = {}
        self.create_subscription(Image, "/isaac_camera/image_raw", self.on_raw, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, "/isaac_camera/jpeg", self.on_jpeg, qos_profile_sensor_data)
        self.create_subscription(String, "/isaac_camera/jpeg_meta", self.on_meta, 10)

    def on_raw(self, m):
        now = time.time()
        self.raw["lat"].append(now - stamp(m.header))
        self.raw["rx"].append(now)
        self.raw["bytes"] += len(m.data)

    def on_meta(self, m):
        d = json.loads(m.data)
        self.enc[round(d["stamp"], 6)] = d["encode_ms"]

    def on_jpeg(self, m):
        now = time.time()
        t0 = time.perf_counter()
        img = cv2.imdecode(np.frombuffer(m.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        dec = time.perf_counter() - t0
        done = time.time()
        s = stamp(m.header)
        self.jpg["lat"].append(now - s)
        self.jpg["rx"].append(now)
        self.jpg["bytes"] += len(m.data)
        self.jpg["decode"].append(dec)
        self.jpg["total"].append(done - s)
        if self.show and img is not None:
            cv2.imshow("operator view", img)
            cv2.waitKey(1)


def pct(x, scale=1000.0):
    x = np.asarray(x)
    if x.size == 0:
        return {"n": 0}
    return {"n": int(x.size), "p50": round(float(np.percentile(x, 50) * scale), 2),
            "p95": round(float(np.percentile(x, 95) * scale), 2),
            "p99": round(float(np.percentile(x, 99) * scale), 2), "max": round(float(x.max() * scale), 2)}


def stream_stats(d, seconds):
    rx = np.array(d["rx"])
    gaps = np.diff(rx) if rx.size > 1 else np.array([])
    return {"fps": round(rx.size / seconds, 1), "mbit_s": round(d["bytes"] * 8 / seconds / 1e6, 2),
            "bytes_per_frame": int(d["bytes"] / max(1, rx.size)), "gap_ms": pct(gaps)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--out", default="/ws/results/video.json")
    ap.add_argument("--show", action="store_true")
    a, _ = ap.parse_known_args()
    rclpy.init()
    n = Measure(a.show)
    end = time.time() + a.seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    enc = np.array(list(n.enc.values())) / 1000.0
    res = {
        "raw": {"render_to_rx_ms": pct(n.raw["lat"]), **stream_stats(n.raw, a.seconds)},
        "jpeg": {"encode_ms": pct(enc), "render_to_rx_ms": pct(n.jpg["lat"]), "decode_ms": pct(n.jpg["decode"]),
                 "render_to_decoded_ms": pct(n.jpg["total"]), **stream_stats(n.jpg, a.seconds)},
    }
    print(json.dumps(res, indent=2))
    with open(a.out, "w") as f:
        json.dump(res, f, indent=2)
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
