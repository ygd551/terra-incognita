"""Build the RMF-Obelix asset for drone_sim.py from ntnu-arl/rotors_simulator's mesh (downloaded by
tools/add_drone.sh): one rigid body whose origin is the frame centre (centre of mass, IMU), mass and inertia from
config/robots.yaml, collision = a box the size of the frame (protective rings included) + the lidar on top, visual =
the RotorS mesh (textured) + a grey cylinder for the Ouster OS0, which the mesh does not include.
  /isaac-sim/python.sh drone_to_usd.py [--robot rmf_obelix_ouster64]   -> assets/drones/<model>/<model>.usd
(one asset per airframe, `model` in config/robots.yaml: the lidar variants share it, Ouster cylinder included)
"""
import argparse
import asyncio
import os

parser = argparse.ArgumentParser()
parser.add_argument("--robot", default="rmf_obelix_ouster64")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import yaml  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("omni.kit.asset_converter", True)
import omni.kit.asset_converter as conv  # noqa: E402

ROOT = "/workspace/isaac_explore"
R = yaml.safe_load(open(f"{ROOT}/config/robots.yaml"))[args.robot]
MODEL = R.get("model", args.robot)
DIR = f"{ROOT}/assets/drones/{MODEL}"
MESH, OUT = f"{DIR}/{MODEL}.dae", f"{DIR}/{MODEL}.usd"
VISUAL = f"{DIR}/mesh/{MODEL}.usd"
# the RotorS mesh's bounds (its origin is at the bottom of the frame): shift it so the frame centre is the body origin
MESH_CENTRE = (-0.004, 0.0, 0.076)

ctx = conv.AssetConverterContext()
ctx.use_meter_as_world_unit = True
ctx.ignore_animations, ctx.ignore_camera, ctx.ignore_light = True, True, True
os.makedirs(os.path.dirname(VISUAL), exist_ok=True)
task = conv.get_instance().create_converter_task(MESH, VISUAL, None, ctx)
if not asyncio.get_event_loop().run_until_complete(task.wait_until_finished()):
    raise SystemExit(f"[drone_to_usd] converting {MESH} failed: {task.get_error_message()}")

stage = Usd.Stage.CreateNew(OUT)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
body = UsdGeom.Xform.Define(stage, f"/{MODEL}")
stage.SetDefaultPrim(body.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
mass = UsdPhysics.MassAPI.Apply(body.GetPrim())
mass.CreateMassAttr(R["mass"])
mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*R["inertia"]))
mass.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))

# collision: the frame box and the lidar, invisible
frame = UsdGeom.Cube.Define(stage, f"/{MODEL}/collision_frame")
frame.CreateSizeAttr(1.0)
frame.AddScaleOp().Set(Gf.Vec3f(*R["frame_size"]))
lidar_z, lidar_r, lidar_h = R["lidar_pos"][2], 0.043, 0.07   # Ouster OS0: 85 mm diameter, ~70 mm tall
lid = UsdGeom.Cylinder.Define(stage, f"/{MODEL}/collision_lidar")
lid.CreateRadiusAttr(lidar_r)
lid.CreateHeightAttr(lidar_h)
lid.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, lidar_z))
for g in (frame, lid):
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    g.CreatePurposeAttr(UsdGeom.Tokens.guide)   # not rendered

# visual: the RotorS mesh and a grey lidar
vis = UsdGeom.Xform.Define(stage, f"/{MODEL}/visual")   # the shift; the converted mesh keeps its own transform
vis.AddTranslateOp().Set(Gf.Vec3d(*(-np.array(MESH_CENTRE))))
vis.AddRotateXOp().Set(90.0)   # the converter turns this Z-up COLLADA file as if it were Y-up (-90 deg about x)
stage.DefinePrim(f"/{MODEL}/visual/mesh", "Xform").GetReferences().AddReference(os.path.relpath(VISUAL, DIR))
cyl = UsdGeom.Cylinder.Define(stage, f"/{MODEL}/visual_lidar")
cyl.CreateRadiusAttr(lidar_r)
cyl.CreateHeightAttr(lidar_h)
cyl.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, lidar_z))
mat = UsdShade.Material.Define(stage, f"/{MODEL}/Looks/lidar")
shader = UsdShade.Shader.Define(stage, f"/{MODEL}/Looks/lidar/shader")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.35, 0.35, 0.38))
mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(mat)
stage.GetRootLayer().Save()
print(f"[drone_to_usd] {OUT}: mass {R['mass']} kg, inertia {R['inertia']}, frame {R['frame_size']} m, "
      f"lidar at {R['lidar_pos']}", flush=True)
app.close()
