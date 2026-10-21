"""Turn a single terrain mesh (e.g. from terrain_gen/) into a scene USD for go2_sim.py.
  /isaac-sim/python.sh mesh_to_usd.py IN.obj OUT.usd [--spawn spawnable_locations.npy]
The mesh becomes /World/collision: visible, static triangle collider, and the single mesh that go2_sim's
RayCasters (height scan + lidar) cast against. With --spawn, prints a spawn point near the map centre."""
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("mesh")
parser.add_argument("out")
parser.add_argument("--spawn", help="terrain-generator spawnable_locations.npy")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import trimesh  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt  # noqa: E402

m = trimesh.load(args.mesh, force="mesh")
stage = Usd.Stage.CreateNew(args.out)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
mesh = UsdGeom.Mesh.Define(stage, "/World/collision")
mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(m.vertices, dtype=np.float32)))
mesh.CreateFaceVertexCountsAttr([3] * len(m.faces))
mesh.CreateFaceVertexIndicesAttr(np.asarray(m.faces).reshape(-1).astype(int).tolist())
mesh.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.53, 0.5)])
UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set("none")  # static triangle mesh
stage.GetRootLayer().Save()
msg = f"[mesh_to_usd] {args.out}: {len(m.faces)} tris, bounds {np.round(m.bounds, 1).tolist()}"
if args.spawn:
    s = np.load(args.spawn)
    p = s[np.argmin(np.linalg.norm(s[:, :2] - m.bounds.mean(0)[:2], axis=1))]
    # terrain-generator stores spawn points 0.5 m above the floor (height_offset=0.5 in generate_with_wfc.py);
    # go2_sim wants the base ~0.42 m above the floor
    msg += f"; spawn near centre: --spawn {p[0]:.2f} {p[1]:.2f} {p[2] - 0.5 + 0.42:.2f}"
print(msg, flush=True)
app.close()
