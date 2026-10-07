"""Isaac Sim Script Editor: operator camera publishing RGB over ROS 2.

Run with the Franka teleop scene loaded and the simulation stopped.
  * Camera /World/TeleopCamera in front of the arm, looking at the workspace.
  * 320x240 render product, published on /isaac_camera/image_raw
    (and camera_info), every FRAME_SKIP+1 rendered frames.
  * Stamped with wall-clock (system) time when the node supports it, so the
    receiver can measure render-to-receipt latency on synced clocks.
"""

import omni.graph.core as og
import omni.usd
import usdrt
from pxr import Gf, UsdGeom

WIDTH, HEIGHT = 320, 240
FRAME_SKIP = 2            # publish every 3rd rendered frame
EYE = Gf.Vec3d(1.6, 0.0, 0.9)
LOOK_AT = Gf.Vec3d(0.4, 0.0, 0.35)
CAM = "/World/TeleopCamera"
GRAPH = "/World/CameraGraph"

stage = omni.usd.get_context().get_stage()

# USD cameras look down -Z with +Y up: the inverse of a look-at view matrix.
cam = UsdGeom.Camera.Define(stage, CAM)
cam.CreateFocalLengthAttr(18.0)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 20.0))
xf = UsdGeom.Xformable(cam.GetPrim())
xf.ClearXformOpOrder()
xf.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(EYE, LOOK_AT, Gf.Vec3d(0, 0, 1)).GetInverse())
print("camera:", CAM, "eye", tuple(EYE), "looking at", tuple(LOOK_AT))

if stage.GetPrimAtPath(GRAPH):
    stage.RemovePrim(GRAPH)
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": GRAPH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("rp", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ],
        keys.CONNECT: [
            ("tick.outputs:tick", "rp.inputs:execIn"),
            ("rp.outputs:execOut", "rgb.inputs:execIn"),
            ("rp.outputs:execOut", "info.inputs:execIn"),
            ("rp.outputs:renderProductPath", "rgb.inputs:renderProductPath"),
            ("rp.outputs:renderProductPath", "info.inputs:renderProductPath"),
            ("ctx.outputs:context", "rgb.inputs:context"),
            ("ctx.outputs:context", "info.inputs:context"),
        ],
        keys.SET_VALUES: [
            ("rp.inputs:cameraPrim", [usdrt.Sdf.Path(CAM)]),
            ("rp.inputs:width", WIDTH),
            ("rp.inputs:height", HEIGHT),
            ("rgb.inputs:topicName", "isaac_camera/image_raw"),
            ("rgb.inputs:type", "rgb"),
            ("rgb.inputs:frameId", "isaac_camera"),
            ("rgb.inputs:frameSkipCount", FRAME_SKIP),
            ("info.inputs:topicName", "isaac_camera/camera_info"),
            ("info.inputs:frameId", "isaac_camera"),
            ("info.inputs:frameSkipCount", FRAME_SKIP),
        ],
    },
)

# Wall-clock stamps: the input name differs between versions, so check.
for name in ("rgb", "info"):
    node = og.Controller.node(f"{GRAPH}/{name}")
    attrs = [a.get_name() for a in node.get_attributes()]
    if "inputs:useSystemTime" in attrs:
        og.Controller.set(node.get_attribute("inputs:useSystemTime"), True)
        print(f"{name}: stamped with system time")
    else:
        print(f"{name}: no useSystemTime input; stamps are sim time. Inputs:",
              [a for a in attrs if a.startswith("inputs:")])
print("camera graph ready:", GRAPH, f"{WIDTH}x{HEIGHT}, every {FRAME_SKIP + 1} frames")
