#!/usr/bin/env python3
"""Records one bag per teleoperation episode, driven by the safety state.

An episode starts when the supervisor enters ACTIVE via "engaged" and ends
on IDLE, FAULT or ESTOP (HOLD stays inside the episode). Each episode gets
its own directory:

  <root>/episode_0003/bag/       rosbag2 (mcap), serialized messages as received
  <root>/episode_0003/meta.json  start/end, duration, end mode and reason,
                                 message counts, gains and operator notes

    python3 episode_recorder.py --root /ws/episodes --note "circle task"
"""

import argparse
import json
import os
import time

import rclpy
import rosbag2_py
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

TOPICS = {
    "/isaac_camera/jpeg": (CompressedImage, "sensor_msgs/msg/CompressedImage", True),
    "/isaac_joint_states_fast": (JointState, "sensor_msgs/msg/JointState", True),
    "/isaac_joint_commands": (JointState, "sensor_msgs/msg/JointState", False),
    "/isaac_ee_tf": (TFMessage, "tf2_msgs/msg/TFMessage", True),
    "/operator/marker_pose": (PoseStamped, "geometry_msgs/msg/PoseStamped", True),
    "/teleop/state": (String, "std_msgs/msg/String", False),
}
END_MODES = {"IDLE", "FAULT", "ESTOP"}


class Recorder(Node):
    def __init__(self, root, note):
        super().__init__("episode_recorder")
        self.root, self.note = root, note
        os.makedirs(root, exist_ok=True)
        self.writer = None
        self.meta = None
        for topic, (cls, _, sensor) in TOPICS.items():
            qos = qos_profile_sensor_data if sensor else 10
            self.create_subscription(cls, topic, lambda data, t=topic: self.on_msg(t, data), qos, raw=True)
        self.create_subscription(String, "/teleop/state", self.on_state, 10)
        self.get_logger().info(f"waiting for engage; episodes go to {root}")

    def next_index(self):
        used = [int(d.split("_")[1]) for d in os.listdir(self.root) if d.startswith("episode_")]
        return max(used, default=0) + 1

    def start(self, t):
        idx = self.next_index()
        d = os.path.join(self.root, f"episode_{idx:04d}")
        os.makedirs(d)
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=os.path.join(d, "bag"), storage_id="mcap"),
                         rosbag2_py.ConverterOptions("", ""))
        for i, (topic, (_, type_name, _)) in enumerate(TOPICS.items()):
            try:  # Jazzy added an id field; older releases reject it
                meta = rosbag2_py.TopicMetadata(id=i, name=topic, type=type_name, serialization_format="cdr")
            except TypeError:
                meta = rosbag2_py.TopicMetadata(name=topic, type=type_name, serialization_format="cdr")
            self.writer.create_topic(meta)
        self.meta = {"episode": idx, "dir": d, "start": t, "note": self.note,
                     "counts": {k: 0 for k in TOPICS}, "transitions": []}
        self.get_logger().info(f"episode {idx} started")

    def stop(self, t, mode, reason):
        self.writer = None  # closes the bag
        m = self.meta
        m.update(end=t, duration_s=round(t - m["start"], 3), end_mode=mode, end_reason=reason,
                 clean_end=(mode == "IDLE" and reason == "disengaged"))
        with open(os.path.join(m["dir"], "meta.json"), "w") as f:
            json.dump(m, f, indent=2)
        self.get_logger().info(f"episode {m['episode']} ended: {mode} ({reason}), {m['duration_s']} s, "
                               f"frames {m['counts']['/isaac_camera/jpeg']}")
        self.meta = None

    def on_msg(self, topic, data):
        if self.writer is None:
            return
        self.writer.write(topic, data, time.time_ns())
        self.meta["counts"][topic] += 1

    def on_state(self, m):
        s = json.loads(m.data)
        if not s.get("transition"):
            return
        mode, reason, t = s["mode"], s["reason"], s["t"]
        if self.meta is not None:
            self.meta["transitions"].append({"t": t, "mode": mode, "reason": reason})
        if self.writer is None and mode == "ACTIVE" and reason == "engaged":
            self.start(t)
            self.meta["transitions"].append({"t": t, "mode": mode, "reason": reason})
        elif self.writer is not None and mode in END_MODES:
            self.stop(t, mode, reason)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/ws/episodes")
    ap.add_argument("--note", default="")
    a, _ = ap.parse_known_args()
    rclpy.init()
    n = Recorder(a.root, a.note)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if n.meta is not None:
            n.stop(time.time(), "INTERRUPTED", "recorder stopped")
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
