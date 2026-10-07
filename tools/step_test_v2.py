#!/usr/bin/env python3
"""Joint step test: separate command delay from joint-drive response.

For each trial: wait until the arm is still, command a small step on one
joint (others held), then read Isaac's joint states, which are stamped with
wall-clock time at the physics step that produced them (chrony-synced).

  onset    command published -> first physics step with measurable motion
           (transport + wait for the next physics step + drive start)
  seen     command published -> that first-motion sample received here
           (the full round trip the controller experiences)
  rise     10% -> 90% of the step (joint-drive dynamics)
  t90      command published -> 90% of the step
  overshoot, settle (within 2% of the step)

With --stream, the step target is republished at 100 Hz for the whole
trial, as the teleop loop does, so a lost packet only delays motion until
the next command gets through. Use it under network loss.

The teleop node must be disengaged, or it will fight the steps.

    python3 step_test_v2.py --joint panda_joint1 --step 0.1 --trials 30 [--stream]
"""

import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

JOINTS = [f"panda_joint{i}" for i in range(1, 8)]


def analyze_trial(t, q, t_pub, q0, step, onset_frac=0.02):
    """t: Isaac stamps (s), q: joint position, both after t_pub.
    Returns a dict of times in seconds relative to t_pub."""
    d = (q - q0) / step  # normalised: 0 before, 1 at target
    out = {}
    hit = np.where(d > onset_frac)[0]
    if hit.size == 0:
        return None
    out["onset"] = t[hit[0]] - t_pub
    i10, i90 = np.where(d >= 0.1)[0], np.where(d >= 0.9)[0]
    if i10.size and i90.size:
        out["rise"] = t[i90[0]] - t[i10[0]]
        out["t90"] = t[i90[0]] - t_pub
    out["overshoot"] = max(0.0, float(d.max() - 1.0))
    outside = np.where(np.abs(d - 1.0) > 0.02)[0]
    if outside.size and outside[-1] + 1 < len(t):
        out["settle"] = t[outside[-1] + 1] - t_pub
    return out


class StepTest(Node):
    def __init__(self):
        super().__init__("step_test")
        self.pub = self.create_publisher(JointState, "/isaac_joint_commands", 10)
        self.create_subscription(JointState, "/isaac_joint_states_fast", self.on_js, qos_profile_sensor_data)
        self.buf = []  # (isaac_stamp, rx_time, positions[7])

    def on_js(self, m):
        try:
            q = [m.position[m.name.index(j)] for j in JOINTS]
        except ValueError:
            return
        self.buf.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, time.time(), q))

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def latest(self):
        return np.array(self.buf[-1][2]) if self.buf else None

    def wait_still(self, tol=2e-4, hold=0.5, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end:
            self.spin_for(hold)
            recent = [b for b in self.buf if b[1] > time.time() - hold]
            if len(recent) > 5:
                qs = np.array([b[2] for b in recent])
                if np.abs(qs - qs[-1]).max() < tol:
                    return qs[-1]
        return self.latest()

    def command(self, q):
        m = JointState()
        m.name = JOINTS
        m.position = [float(v) for v in q]
        t = time.time()
        m.header.stamp.sec, m.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
        self.pub.publish(m)
        return time.time()


def pct(x, scale=1000.0):
    x = np.asarray([v for v in x if v is not None])
    if x.size == 0:
        return {"n": 0}
    return {"n": int(x.size), "p50": round(float(np.percentile(x, 50) * scale), 1),
            "p95": round(float(np.percentile(x, 95) * scale), 1),
            "max": round(float(x.max() * scale), 1)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="panda_joint1", choices=JOINTS)
    ap.add_argument("--step", type=float, default=0.1, help="rad")
    ap.add_argument("--trials", type=int, default=30)
    ap.add_argument("--window", type=float, default=2.0, help="s recorded per trial")
    ap.add_argument("--out", default="/ws/results/step_test.json")
    ap.add_argument("--stream", action="store_true", help="republish the target at 100 Hz during each trial")
    ap.add_argument("--label", default="", help="condition name stored in the output")
    a = ap.parse_args()
    j = JOINTS.index(a.joint)

    rclpy.init()
    node = StepTest()
    for _ in range(20):  # discovery across machines can take seconds
        node.spin_for(0.5)
        if node.buf:
            break
    if not node.buf:
        print("no /isaac_joint_states_fast; is Isaac playing?")
        return
    q_hold = node.wait_still()
    node.command(q_hold)  # hold the current pose explicitly first
    results = []
    for k in range(a.trials):
        q0_all = node.wait_still()
        q0 = q0_all[j]
        step = a.step if k % 2 == 0 else -a.step
        target = q0_all.copy()
        target[j] = q0 + step
        node.buf.clear()
        t_pub = node.command(target)
        if a.stream:
            end = t_pub + a.window
            nxt = t_pub + 0.01
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=max(0.0, nxt - time.time()))
                if time.time() >= nxt:
                    node.command(target)
                    nxt += 0.01
        else:
            node.spin_for(a.window)
        rows = [b for b in node.buf if b[0] >= t_pub - 0.5]
        t = np.array([b[0] for b in rows])
        rx = np.array([b[1] for b in rows])
        q = np.array([b[2][j] for b in rows])
        after = t >= t_pub - 0.002  # allow for clock offset
        r = analyze_trial(t[after], q[after], t_pub, q0, step)
        if r is None:
            print(f"trial {k + 1}: no motion detected")
            continue
        hit = np.where(after)[0][np.argmax((q[after] - q0) / step > 0.02)]
        r["seen"] = rx[hit] - t_pub
        results.append(r)
        print(f"trial {k + 1:2d}: onset {r['onset'] * 1000:6.1f} ms  seen {r['seen'] * 1000:6.1f} ms  "
              f"rise {r.get('rise', float('nan')) * 1000:6.1f} ms  t90 {r.get('t90', float('nan')) * 1000:6.1f} ms  "
              f"overshoot {r['overshoot'] * 100:4.1f}%")

    summary = {"label": a.label, "stream": a.stream, "joint": a.joint, "step_rad": a.step,
               "trials": len(results), "no_motion": a.trials - len(results)}
    for key in ("onset", "seen", "rise", "t90", "settle"):
        summary[key + "_ms"] = pct([r.get(key) for r in results])
    summary["overshoot_pct"] = pct([r["overshoot"] for r in results], scale=100.0)
    print(json.dumps(summary, indent=2))
    with open(a.out, "w") as f:
        json.dump(summary, f, indent=2)
    node.command(q_hold)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
