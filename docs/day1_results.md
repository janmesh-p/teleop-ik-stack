# Day 1 results

Setup: Isaac Sim 5.0 on an Ubuntu 24.04 laptop (RTX 5090), Jetson Orin Nano
(JetPack 6, ROS 2 Jazzy in Docker), Intel RealSense D435 on the Jetson.

## Link between laptop and Jetson

| Link | Ping avg | Ping max | Jitter (mdev) |
|---|---|---|---|
| Wi-Fi, same router | 43.4 ms | 324.0 ms | 85.0 ms |
| **USB-C device networking** | **0.35 ms** | **1.01 ms** | **0.21 ms** |

ROS 2 is pinned to the USB link with a Fast DDS profile (`config/fastdds_usb_*.xml`).

## Clock sync

chrony, laptop as server, Jetson as client over USB: offset 9 µs, RMS offset
0.48 ms and falling at the time of measurement. All cross-machine latency
numbers carry this uncertainty.

## Isaac Sim arm interface

- Commands: position targets on `/isaac_joint_commands` move the arm.
- Measured joint effort is available (gravity load: joint 2 -6.6 N·m, joint 4 18.7 N·m).
- Default graph publishes joint states once per rendered frame, stamped with sim time.

| Configuration | Joint-state rate | Worst gap |
|---|---|---|
| Per rendered frame, 2582×1372 viewport | 34 Hz | ~35 ms |
| Per physics step (120 Hz), 2582×1372 | 68 Hz, bursty | 35 ms |
| **Per physics step, 1280×720 viewport** | **102 Hz** | **21 ms** |

Finding: feedback rate is bound by the render loop, not the ROS bridge. Physics
steps run in batches per rendered frame, so states arrive in bursts.

## Operator marker pose (RealSense D435 on Jetson, 640×480 at 60 fps)

| Stage | p50 | p95 | p99 |
|---|---|---|---|
| Capture → frame received | 18.8 ms | 19.3 ms | 19.8 ms |
| Detection + pose (Orin CPU) | 3.4 ms | 4.0 ms | 4.7 ms |
| **Capture → pose** | **22.2 ms** | **23.2 ms** | **24.0 ms** |

Pose noise with the marker held still: under 1 mm. The capture-to-receive time
is roughly one frame period: exposure, readout and USB transfer. Fixed short
exposure is a planned test.
