"""Isaac Sim Script Editor: scale the Franka's joint drive gains.

Joint drives are springs with dampers: stiffness pulls toward the target,
damping resists speed. Scaling stiffness by F and damping by sqrt(F) keeps
the damping ratio (how much it overshoots) unchanged and makes the response
about sqrt(F) times faster.

Run with the simulation stopped. Set FACTOR = 1.0 to only print the gains.
Gains are read fresh each run, so running twice compounds the scaling:
reload the scene to start over.
"""

import math

import omni.usd
from pxr import UsdPhysics

FACTOR = 10.0

stage = omni.usd.get_context().get_stage()
joints = sorted(
    (p for p in stage.Traverse() if p.GetName().startswith("panda_joint") and p.GetName()[-1].isdigit()
     and p.HasAPI(UsdPhysics.DriveAPI, "angular")),
    key=lambda p: p.GetName())
for p in joints:
    drive = UsdPhysics.DriveAPI.Get(p, "angular")
    k, c = drive.GetStiffnessAttr().Get(), drive.GetDampingAttr().Get()
    if FACTOR != 1.0:
        drive.GetStiffnessAttr().Set(k * FACTOR)
        drive.GetDampingAttr().Set(c * math.sqrt(FACTOR))
    k2, c2 = drive.GetStiffnessAttr().Get(), drive.GetDampingAttr().Get()
    print(f"{p.GetName()}: stiffness {k:.1f} -> {k2:.1f}, damping {c:.1f} -> {c2:.1f}")
print(f"{len(joints)} drives, factor {FACTOR}")
