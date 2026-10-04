"""Isaac Sim Script Editor: run joint I/O on every physics step, not every frame.

Run with the Franka ROS 2 scene loaded and the simulation stopped.
  * Physics at 120 Hz.
  * Disables the scene's per-frame articulation controller, so only one
    controller drives the arm.
  * New graph on the physics step:
      publishes /isaac_joint_states_fast, stamped with wall-clock time
      applies /isaac_joint_commands to the arm
"""

import omni.graph.core as og
import omni.usd
from pxr import PhysxSchema, UsdPhysics

PHYSICS_HZ = 120
GRAPH = "/World/PhysicsRateGraph"

stage = omni.usd.get_context().get_stage()
robot = next(p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI))
scene = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
PhysxSchema.PhysxSceneAPI.Apply(scene).CreateTimeStepsPerSecondAttr().Set(PHYSICS_HZ)
print("robot:", robot.GetPath(), "| physics scene:", scene.GetPath(), f"-> {PHYSICS_HZ} Hz")

for g in og.get_all_graphs():
    for n in g.get_nodes():
        if "ArticulationController" in n.get_type_name() and not str(n.get_prim_path()).startswith(GRAPH):
            n.get_attribute("inputs:robotPath").set("")
            print("disabled old controller:", n.get_prim_path())

if stage.GetPrimAtPath(GRAPH):
    stage.RemovePrim(GRAPH)
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": GRAPH, "evaluator_name": "execution",
     "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
    {
        keys.CREATE_NODES: [
            ("step", "isaacsim.core.nodes.OnPhysicsStep"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("wall", "isaacsim.core.nodes.IsaacReadSystemTime"),
            ("pub", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("sub", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ctrl", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        keys.CONNECT: [
            ("step.outputs:step", "pub.inputs:execIn"),
            ("step.outputs:step", "sub.inputs:execIn"),
            ("step.outputs:step", "ctrl.inputs:execIn"),
            ("ctx.outputs:context", "pub.inputs:context"),
            ("ctx.outputs:context", "sub.inputs:context"),
            ("wall.outputs:systemTime", "pub.inputs:timeStamp"),
            ("sub.outputs:jointNames", "ctrl.inputs:jointNames"),
            ("sub.outputs:positionCommand", "ctrl.inputs:positionCommand"),
            ("sub.outputs:velocityCommand", "ctrl.inputs:velocityCommand"),
            ("sub.outputs:effortCommand", "ctrl.inputs:effortCommand"),
        ],
        keys.SET_VALUES: [
            ("pub.inputs:topicName", "isaac_joint_states_fast"),
            ("pub.inputs:targetPrim", str(robot.GetPath())),
            ("sub.inputs:topicName", "isaac_joint_commands"),
            ("ctrl.inputs:robotPath", str(robot.GetPath())),
        ],
    },
)
print("physics-rate graph ready:", GRAPH)
