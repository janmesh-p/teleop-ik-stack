#!/usr/bin/env python3
"""Exports a recorded episode to a synchronized learning dataset.

One row per camera frame (the slowest stream), at the frame's render stamp t:
  observation  image (frames.mp4, row i = frame i), joint position, velocity
               and effort, hand pose, operator marker pose, safety mode,
               each the latest sample with stamp <= t
  action       joint position command: the first command with stamp >= t
  ages         how old each observation was at t, and how far ahead the
               action was, in ms, so a learner can filter stale rows

Rows without a joint state yet, or without a following command, are dropped.

    python3 export_episode.py /ws/episodes/episode_0001
"""

import json
import os
import sys

import cv2
import numpy as np
import rosbag2_py
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from align import first_at_or_after, latest_at_or_before

JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
MODES = ["IDLE", "ACTIVE", "HOLD", "FAULT", "ESTOP"]


def st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def pick(m):
    idx = [m.name.index(j) for j in JOINTS]
    g = lambda arr: [arr[i] for i in idx] if len(arr) == len(m.name) else [np.nan] * 7
    return g(m.position), g(m.velocity), g(m.effort)


def main(ep):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=os.path.join(ep, "bag"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    img_t, imgs = [], []
    js_t, js_p, js_v, js_e = [], [], [], []
    cmd_t, cmd_p = [], []
    ee_t, ee = [], []
    mk_t, mk = [], []
    md_t, md = [], []
    while reader.has_next():
        topic, data, t_rec = reader.read_next()
        if topic == "/isaac_camera/jpeg":
            m = deserialize_message(data, CompressedImage)
            img_t.append(st(m.header)); imgs.append(bytes(m.data))
        elif topic == "/isaac_joint_states_fast":
            m = deserialize_message(data, JointState)
            p, v, e = pick(m)
            js_t.append(st(m.header)); js_p.append(p); js_v.append(v); js_e.append(e)
        elif topic == "/isaac_joint_commands":
            m = deserialize_message(data, JointState)
            if all(j in m.name for j in JOINTS):
                cmd_t.append(st(m.header) or t_rec * 1e-9); cmd_p.append(pick(m)[0])
        elif topic == "/isaac_ee_tf":
            for tf in deserialize_message(data, TFMessage).transforms:
                if tf.child_frame_id == "panda_hand":
                    a, r = tf.transform.translation, tf.transform.rotation
                    ee_t.append(st(tf.header)); ee.append([a.x, a.y, a.z, r.x, r.y, r.z, r.w])
        elif topic == "/operator/marker_pose":
            m = deserialize_message(data, PoseStamped)
            a, r = m.pose.position, m.pose.orientation
            mk_t.append(st(m.header)); mk.append([a.x, a.y, a.z, r.x, r.y, r.z, r.w])
        elif topic == "/teleop/state":
            s = json.loads(deserialize_message(data, String).data)
            md_t.append(s["t"]); md.append(MODES.index(s["mode"]) if s["mode"] in MODES else -1)

    def srt(t, *xs):
        o = np.argsort(t)
        return (np.asarray(t)[o],) + tuple(np.asarray(x)[o] for x in xs)

    img_t, imgs = srt(img_t, np.arange(len(imgs)))[0], [imgs[i] for i in np.argsort(img_t)]
    js_t, js_p, js_v, js_e = srt(js_t, js_p, js_v, js_e)
    cmd_t, cmd_p = srt(cmd_t, cmd_p)
    ee_t, ee = srt(ee_t, ee)
    mk_t, mk = srt(mk_t, mk)
    md_t, md = srt(md_t, md)

    i_js = latest_at_or_before(js_t, img_t)
    i_cmd = first_at_or_after(cmd_t, img_t)
    keep = (i_js >= 0) & (i_cmd < len(cmd_t))
    rows = np.where(keep)[0]
    if rows.size == 0:
        print("no usable rows"); return

    def obs(t_src, x, fill_dim):
        i = latest_at_or_before(t_src, img_t[rows]) if len(t_src) else np.full(rows.size, -1)
        out = np.full((rows.size, fill_dim), np.nan)
        age = np.full(rows.size, np.nan)
        ok = i >= 0
        if ok.any():
            out[ok] = np.asarray(x)[i[ok]].reshape(ok.sum(), fill_dim)
            age[ok] = (img_t[rows][ok] - t_src[i[ok]]) * 1000
        return out, age

    ee_o, ee_age = obs(ee_t, ee, 7)
    mk_o, mk_age = obs(mk_t, mk, 7)
    md_o, _ = obs(md_t, md, 1)
    data = {
        "t": img_t[rows],
        "joint_pos": js_p[i_js[rows]], "joint_vel": js_v[i_js[rows]], "joint_effort": js_e[i_js[rows]],
        "hand_pose": ee_o, "marker_pose": mk_o, "safety_mode": md_o[:, 0],
        "action_joint_pos": cmd_p[i_cmd[rows]],
        "joint_age_ms": (img_t[rows] - js_t[i_js[rows]]) * 1000,
        "action_lead_ms": (cmd_t[i_cmd[rows]] - img_t[rows]) * 1000,
        "hand_age_ms": ee_age, "marker_age_ms": mk_age,
    }
    np.savez_compressed(os.path.join(ep, "data.npz"), **data)

    first = cv2.imdecode(np.frombuffer(imgs[rows[0]], np.uint8), cv2.IMREAD_COLOR)
    h, w = first.shape[:2]
    fps = max(1.0, (rows.size - 1) / max(1e-6, img_t[rows][-1] - img_t[rows][0]))
    vw = cv2.VideoWriter(os.path.join(ep, "frames.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    for i in rows:
        vw.write(cv2.imdecode(np.frombuffer(imgs[i], np.uint8), cv2.IMREAD_COLOR))
    vw.release()

    meta_p = os.path.join(ep, "meta.json")
    meta = json.load(open(meta_p)) if os.path.exists(meta_p) else {}
    q = lambda x: {k: round(float(np.nanpercentile(x, p)), 2) for k, p in (("p50", 50), ("p99", 99))}
    meta["export"] = {"rows": int(rows.size), "frames_dropped": int(len(img_t) - rows.size), "fps": round(fps, 2),
                      "joint_age_ms": q(data["joint_age_ms"]), "action_lead_ms": q(data["action_lead_ms"]),
                      "modes": MODES, "fields": {k: list(v.shape) for k, v in data.items()}}
    json.dump(meta, open(meta_p, "w"), indent=2)
    print(json.dumps(meta["export"], indent=2))


if __name__ == "__main__":
    main(sys.argv[1])
