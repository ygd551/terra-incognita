"""Test scene for the walking policy on slopes: lanes 10 m apart (y = 0, 10, ...), one per angle, each flat ground ->
ramp up RISE m -> 3 m plateau -> ramp down at the same angle -> flat ground, 4 m wide. Same layout as sim/sdf_to_usd.py
output (default prim /World, the one static triangle collider at /World/collision, which go2_sim's height scanner and
lidar ray-cast), so go2_sim.py --scene takes it.
  /workspace/isaaclab/isaaclab.sh -p make_ramps.py OUT.usd"""
import sys

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, UsdPhysics, Vt  # noqa: E402

ANGLES = [10, 15, 20, 25, 30, 35]
RISE, TOP, HALF_W, LANE = 1.5, 3.0, 2.0, 10.0


def ramp_lane(y, angle):
    """Points and triangles of one up-plateau-down ramp (a prism along x starting at x = 0)."""
    d = RISE / np.tan(np.radians(angle))
    prof = [(0.0, 0.0), (d, RISE), (d + TOP, RISE), (2 * d + TOP, 0.0)]   # (x, z) along the lane
    pts = [(x, y + s * HALF_W, z) for s in (-1, 1) for x, z in prof]      # 0..3 right side, 4..7 left side
    tris = []
    for i in range(3):                       # top surface strips
        tris += [(i, i + 1, i + 5), (i, i + 5, i + 4)]
    for o in (0, 4):                         # the two side walls (quad 0-1-2-3 down to z = 0: closed by the ground)
        tris += [(o, o + 1, o + 2), (o, o + 2, o + 3)]
    return pts, tris


pts, tris = [(-10.0, -6.0, 0.0), (60.0, -6.0, 0.0), (60.0, 6.0 + LANE * (len(ANGLES) - 1), 0.0),
             (-10.0, 6.0 + LANE * (len(ANGLES) - 1), 0.0)], [(0, 1, 2), (0, 2, 3)]   # ground
for k, a in enumerate(ANGLES):
    p, t = ramp_lane(LANE * k, a)
    tris += [tuple(i + len(pts) for i in tri) for tri in t]
    pts += p

stage = Usd.Stage.CreateNew(sys.argv[1])
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
mesh = UsdGeom.Mesh.Define(stage, "/World/collision")
mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.array(pts, np.float32)))
mesh.CreateFaceVertexCountsAttr([3] * len(tris))
mesh.CreateFaceVertexIndicesAttr([i for tri in tris for i in tri])
mesh.CreateDoubleSidedAttr(True)
UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set("none")
stage.GetRootLayer().Save()
print("[DONE] %s: %d lanes %s deg, %d tris" % (sys.argv[1], len(ANGLES), ANGLES, len(tris)), flush=True)
app.close()
