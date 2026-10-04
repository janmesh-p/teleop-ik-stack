#!/usr/bin/env python3
"""Teleop session analysis: true tracking error and end-to-end latency.

Inputs (from one session):
  * the teleop node's per-cycle CSV log
  * a ROS 2 bag with /isaac_ee_tf and /isaac_joint_states_fast

All stamps are wall-clock seconds on chrony-synced clocks.

Latency method: the Isaac hand trajectory is a delayed copy of the target.
For each window, both velocity signals are resampled at 1 kHz and the shift
that maximises their normalised cross-correlation is the lag. Windows with
too little motion are skipped, since a still hand has no lag to measure.

    python3 analyze_session.py --csv /ws/results/s1.csv --bag /ws/results/s1_bag --out /ws/results/s1
"""

import argparse
import csv
import json

import numpy as np

FS = 1000.0  # resampling rate for lag estimation, Hz


SMOOTH_S = 0.05  # velocity smoothing window


def _smooth(x, n):
    k = np.ones(n) / n
    return np.column_stack([np.convolve(x[:, i], k, mode="same") for i in range(x.shape[1])])


def estimate_lag(t_a, a, t_b, b, max_lag=0.4, min_speed=0.02):
    """Lag (s) by which signal b follows a. Both are (N, 3) positions.

    Returns (lag, peak correlation) or (None, None) if there is too little
    motion. Velocities are used so constant offsets do not bias the result.
    Isaac runs physics steps in bursts (several steps within ~1 ms of wall
    time, then a pause), so the hand moves in jumps against wall time. Both
    velocity signals are smoothed over SMOOTH_S before correlating; without
    this, peak correlation on bursty data falls to ~0.2.
    """
    t0, t1 = max(t_a[0], t_b[0]), min(t_a[-1], t_b[-1])
    if t1 - t0 < 4 * max_lag:
        return None, None
    grid = np.arange(t0, t1, 1.0 / FS)
    va = np.gradient(np.column_stack([np.interp(grid, t_a, a[:, k]) for k in range(3)]), axis=0) * FS
    vb = np.gradient(np.column_stack([np.interp(grid, t_b, b[:, k]) for k in range(3)]), axis=0) * FS
    n_s = max(1, int(SMOOTH_S * FS))
    va, vb = _smooth(va, n_s), _smooth(vb, n_s)
    if np.percentile(np.linalg.norm(va, axis=1), 90) < min_speed:
        return None, None
    n = int(max_lag * FS)
    core = slice(n, len(grid) - n)
    a_core = va[core]
    best, best_lag = -np.inf, 0
    norm_a = np.sqrt((a_core ** 2).sum())
    for lag in range(0, n + 1):
        b_shift = vb[n + lag: len(grid) - n + lag]
        c = (a_core * b_shift).sum() / (norm_a * np.sqrt((b_shift ** 2).sum()) + 1e-12)
        if c > best:
            best, best_lag = c, lag
    return best_lag / FS, best


MAX_TARGET_GAP_S = 0.05  # target samples further apart than this mark a marker dropout


def live_mask(t_target, t_query):
    """True where t_query falls between two target samples less than
    MAX_TARGET_GAP_S apart, i.e. while the marker was actually visible."""
    idx = np.searchsorted(t_target, t_query)
    ok = (idx > 0) & (idx < len(t_target))
    gap = np.full(len(t_query), np.inf)
    gap[ok] = t_target[idx[ok]] - t_target[idx[ok] - 1]
    return gap < MAX_TARGET_GAP_S


def windowed_lags(t_a, a, t_b, b, window=2.0, step=1.0, min_corr=0.8, stats=None):
    lags = []
    tried = rejected_still = rejected_corr = rejected_gap = 0
    t = max(t_a[0], t_b[0])
    end = min(t_a[-1], t_b[-1])
    while t + window <= end:
        ma = (t_a >= t) & (t_a < t + window)
        mb = (t_b >= t - 0.5) & (t_b < t + window + 0.5)
        if ma.sum() > 50 and mb.sum() > 50:
            tried += 1
            if np.diff(t_a[ma]).max() > 2 * MAX_TARGET_GAP_S:
                rejected_gap += 1  # a dropout inside the window would bias the lag
                t += step
                continue
            lag, corr = estimate_lag(t_a[ma], a[ma], t_b[mb], b[mb])
            if lag is None:
                rejected_still += 1
            elif corr < min_corr:
                rejected_corr += 1
            else:
                lags.append(lag)
        t += step
    if stats is not None:
        stats.update(windows=tried, too_still=rejected_still, low_correlation=rejected_corr, dropout=rejected_gap)
    return np.array(lags)


def pct(x, scale=1000.0):
    x = np.asarray(x)
    if x.size == 0:
        return {"n": 0}
    return {"n": int(x.size), "p50": round(float(np.percentile(x, 50) * scale), 2),
            "p95": round(float(np.percentile(x, 95) * scale), 2),
            "p99": round(float(np.percentile(x, 99) * scale), 2),
            "max": round(float(x.max() * scale), 2)}


def read_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    col = lambda k: np.array([float(r[k]) for r in rows])
    return {
        "t_cycle": col("t_cycle"), "t_cap": col("t_marker_capture"), "t_rx": col("t_marker_rx"),
        "stale": col("stale").astype(bool), "clamped": col("clamped").astype(bool),
        "target": np.column_stack([col("target_x"), col("target_y"), col("target_z")]),
        "hand_cmd": np.column_stack([col("hand_x"), col("hand_y"), col("hand_z")]),
        "t_pub": col("t_publish"), "ik_us": col("ik_us"),
    }


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path), rosbag2_py.ConverterOptions("", ""))
    ee_t, ee_p, js_lat = [], [], []
    while reader.has_next():
        topic, data, t_rec_ns = reader.read_next()
        if topic == "/isaac_ee_tf":
            for tf in deserialize_message(data, TFMessage).transforms:
                if tf.child_frame_id == "panda_hand":
                    s = tf.header.stamp
                    ee_t.append(s.sec + s.nanosec * 1e-9)
                    p = tf.transform.translation
                    ee_p.append((p.x, p.y, p.z))
        elif topic == "/isaac_joint_states_fast":
            s = deserialize_message(data, JointState).header.stamp
            js_lat.append(t_rec_ns * 1e-9 - (s.sec + s.nanosec * 1e-9))
    order = np.argsort(ee_t)
    return np.array(ee_t)[order], np.array(ee_p)[order], np.array(js_lat)


def analyze(log, ee_t, ee_p, js_lat):
    live = ~log["stale"]
    res = {}
    # Per-stage timing from stamps.
    fresh = live & (np.diff(np.concatenate([[0.0], log["t_cap"]])) > 0)  # first cycle after each new marker
    res["capture_to_marker_rx_ms"] = pct(log["t_rx"][fresh] - log["t_cap"][fresh])
    res["marker_rx_to_command_ms"] = pct(log["t_pub"][fresh] - log["t_rx"][fresh])
    res["ik_step_us"] = pct(log["ik_us"] * 1e-6, scale=1e6)
    res["isaac_to_jetson_transport_ms"] = pct(js_lat)

    # Lags by trajectory matching.
    t_cmd, p_cmd = log["t_pub"][live], log["hand_cmd"][live]
    t_tgt, p_tgt = log["t_cap"][live], log["target"][live]
    _, first = np.unique(t_tgt, return_index=True)  # one sample per marker frame
    t_tgt, p_tgt = t_tgt[first], p_tgt[first]
    st1, st2 = {}, {}
    res["command_to_isaac_motion_ms"] = pct(windowed_lags(t_cmd, p_cmd, ee_t, ee_p, stats=st1))
    res["command_to_isaac_motion_ms"]["windows"] = st1
    res["capture_to_isaac_motion_ms"] = pct(windowed_lags(t_tgt, p_tgt, ee_t, ee_p, stats=st2))
    res["capture_to_isaac_motion_ms"]["windows"] = st2

    # True tracking error: Isaac hand vs target at the same instant, and
    # the same comparison after removing the measured end-to-end lag.
    # Scored only while the marker is visible: during a dropout the arm holds
    # still and comparing it to a line drawn across the gap is meaningless.
    m = (ee_t > t_tgt[0]) & (ee_t < t_tgt[-1])
    m &= live_mask(t_tgt, ee_t)
    tgt_at_ee = np.column_stack([np.interp(ee_t[m], t_tgt, p_tgt[:, k]) for k in range(3)])
    err = np.linalg.norm(ee_p[m] - tgt_at_ee, axis=1)
    res["tracking_error_mm"] = pct(err, scale=1000.0)
    lag50 = res["capture_to_isaac_motion_ms"].get("p50")
    if lag50 is not None:
        t_back = ee_t - lag50 / 1000.0
        ml = (t_back > t_tgt[0]) & (t_back < t_tgt[-1]) & live_mask(t_tgt, t_back)
        tgt_lagged = np.column_stack([np.interp(t_back[ml], t_tgt, p_tgt[:, k]) for k in range(3)])
        res["tracking_error_lag_removed_mm"] = pct(np.linalg.norm(ee_p[ml] - tgt_lagged, axis=1), scale=1000.0)
    res["session_s"] = round(float(log["t_cycle"][-1] - log["t_cycle"][0]), 1)
    res["stale_fraction"] = round(float(log["stale"].mean()), 3)
    res["clamped_cycles"] = int(log["clamped"].sum())
    return res, (ee_t, ee_p, t_tgt, p_tgt)


def plot(out, series):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ee_t, ee_p, t_tgt, p_tgt = series
    t0 = t_tgt[0]
    gaps = np.where(np.diff(t_tgt) > MAX_TARGET_GAP_S)[0] + 1
    t_plot = np.insert(t_tgt, gaps, np.nan)
    p_plot = np.insert(p_tgt, gaps, np.nan, axis=0)
    m = (ee_t > t_tgt[0]) & (ee_t < t_tgt[-1])
    ee_t, ee_p = ee_t[m], ee_p[m]
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for k, name in enumerate("xyz"):
        axes[k].plot(t_plot - t0, p_plot[:, k], label="target (marker), gaps = marker out of view", lw=1)
        axes[k].plot(ee_t - t0, ee_p[:, k], label="Isaac hand", lw=1)
        axes[k].set_ylabel(f"{name} (m)")
    axes[0].legend(loc="upper right")
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(out + "_tracking.png", dpi=120)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    log = read_csv(a.csv)
    ee_t, ee_p, js_lat = read_bag(a.bag)
    res, series = analyze(log, ee_t, ee_p, js_lat)
    print(json.dumps(res, indent=2))
    with open(a.out + "_results.json", "w") as f:
        json.dump(res, f, indent=2)
    try:
        plot(a.out, series)
        print("plot:", a.out + "_tracking.png")
    except ImportError:
        print("matplotlib missing, no plot")


if __name__ == "__main__":
    main()
