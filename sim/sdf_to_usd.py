"""Convert a Gazebo/Ignition SDF world (DARPA SubT tiles) into a USD stage for Isaac Sim.

Run inside the isaac_explore container (setup.sh does it for simple_cave_01; tools/fetch_assets.py downloads the world
and the Fuel models it uses into assets/):
  /isaac-sim/python.sh /workspace/isaac_explore/sim/sdf_to_usd.py /workspace/isaac_explore/assets/subt/worlds/simple_cave_01.sdf
Output: sim/scenes/<world>.usd, meshes cached in sim/scenes/meshes/ (converted once, reused by all worlds).

Only <include> models are handled (that is all SubT worlds use). Each model's visual and collision
meshes are referenced at their composed SDF pose; collision meshes become static triangle colliders.
"""
import argparse
import asyncio
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser()
parser.add_argument("world", help="path to .sdf world")
parser.add_argument("--fuel", default="/workspace/isaac_explore/assets/fuel", help="ignition fuel cache root")
parser.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenes"))
parser.add_argument("--skip", default="base station|fiducial|backpack|randy|phone|samsung|helmet|rope|vent|gas|drill|extinguisher|cube",
                    help="regex (case-insensitive) of model names to leave out (artifacts / team base)")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

# setup.sh runs this as the very first Isaac Sim start, which compiles the RTX pipelines (cached afterwards): ~21 GB
# of RAM; compiling them one at a time saves ~1 GB of that peak at no extra time
app = SimulationApp({"headless": True, "extra_args": ["--/rtx-transient/psoDb/serializeAllCompiles=true"]})

import omni.kit.app  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt  # noqa: E402

ext_mgr = omni.kit.app.get_app().get_extension_manager()
ext_mgr.set_extension_enabled_immediate("omni.kit.asset_converter", True)
import omni.kit.asset_converter as conv  # noqa: E402


def pose_to_matrix(text):
    """SDF pose 'x y z roll pitch yaw' -> Gf.Matrix4d (row-vector convention, as USD uses)."""
    v = [float(x) for x in (text or "0 0 0 0 0 0").split()]
    x, y, z, r, p, yw = (v + [0] * 6)[:6]
    rot = (Gf.Rotation(Gf.Vec3d(1, 0, 0), math.degrees(r)) *
           Gf.Rotation(Gf.Vec3d(0, 1, 0), math.degrees(p)) *
           Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yw)))  # extrinsic X, Y, Z = Rz*Ry*Rx
    m = Gf.Matrix4d().SetRotate(rot)
    m.SetTranslateOnly(Gf.Vec3d(x, y, z))
    return m


def find_model_dir(uri):
    """Fuel URI -> newest cached version dir, e.g. /fuel/fuel.ignitionrobotics.org/openrobotics/models/<name>/<ver>."""
    m = re.search(r"https?://([^/]+)/[^/]+/([^/]+)/models/([^/]+)", uri)
    if not m:
        return None
    owner, name = m.group(2).lower(), m.group(3).lower()
    for host in (m.group(1), "fuel.ignitionrobotics.org", "fuel.gazebosim.org"):
        d = os.path.join(args.fuel, host, owner, "models", name)
        if os.path.isdir(d):
            vers = sorted((v for v in os.listdir(d) if v.isdigit()), key=int)
            if vers:
                return os.path.join(d, vers[-1])
    return None


def resolve_mesh(uri, mdir):
    """Mesh URI -> local file. Either relative to the model dir, or a full fuel URL
    '.../models/<name>/<ver>/files/<path>' pointing into another (or the same) cached model."""
    m = re.match(r"https?://.+?/models/[^/]+/\d+/files/(.+)", uri)
    if m:
        return os.path.join(find_model_dir(uri) or mdir, m.group(1))
    return os.path.join(mdir, uri.replace("model://", "").split("/", 1)[-1] if uri.startswith("model://") else uri)


_converted = {}


def convert_mesh(src):
    """DAE/OBJ/STL -> USD (cached on disk). Returns usd path or None."""
    if src in _converted:
        return _converted[src]
    rel = os.path.relpath(src, args.fuel)
    dst = os.path.join(args.out, "meshes", os.path.splitext(rel)[0] + ".usd")
    if not os.path.exists(dst):
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        ctx = conv.AssetConverterContext()
        ctx.use_meter_as_world_unit = True  # SDF is in meters
        ctx.ignore_materials = False
        ctx.ignore_animations = True
        ctx.ignore_camera = True
        ctx.ignore_light = True
        task = conv.get_instance().create_converter_task(src, dst, None, ctx)
        ok = asyncio.get_event_loop().run_until_complete(task.wait_until_finished())
        if not ok:
            print(f"[WARN] convert failed: {src}: {task.get_error_message()}", flush=True)
            dst = None
    _converted[src] = dst
    return dst


def dae_unit(f):
    """COLLADA <unit meter=..> (SubT tiles are authored in cm). Gazebo applies it; the asset converter does not."""
    if not f.lower().endswith(".dae"):
        return 1.0
    with open(f, errors="ignore") as fh:
        m = re.search(r'<unit[^>]*meter="([0-9.eE+-]+)"', fh.read(4096))
    return float(m.group(1)) if m else 1.0


def add_mesh(stage, path, usd_file, xform, scale, collide, visible):
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(usd_file)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(Gf.Matrix4d().SetScale(Gf.Vec3d(*scale)) * xform)
    if not visible:
        UsdGeom.Imageable(prim).MakeInvisible()
    if collide:
        collide_roots.append(prim)


def merge_colliders(stage, roots, path):
    """Bake every collision mesh (world frame, triangulated) into ONE static triangle mesh.
    Isaac Lab's RayCaster (height scan for the locomotion policy) only supports a single mesh prim,
    and one big static collider is also cheaper for PhysX than hundreds of referenced ones."""
    import numpy as np
    cache = UsdGeom.XformCache()
    pts_all, tris_all, off = [], [], 0
    for root in roots:
        for p in Usd.PrimRange(root):
            if not p.IsA(UsdGeom.Mesh):
                continue
            m = UsdGeom.Mesh(p)
            pts = np.asarray(m.GetPointsAttr().Get(), dtype=np.float64)
            counts = np.asarray(m.GetFaceVertexCountsAttr().Get())
            idx = np.asarray(m.GetFaceVertexIndicesAttr().Get())
            if len(pts) == 0 or len(counts) == 0:
                continue
            T = np.array(cache.GetLocalToWorldTransform(p))  # row-vector convention
            pts = pts @ T[:3, :3] + T[3, :3]
            starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
            tris = [np.stack([idx[s0] * np.ones(c - 2, int), idx[s0 + 1:s0 + c - 1], idx[s0 + 2:s0 + c]], 1)
                    for s0, c in zip(starts, counts) if c >= 3]  # fan triangulation
            pts_all.append(pts)
            tris_all.append(np.concatenate(tris) + off)
            off += len(pts)
    pts, tris = np.concatenate(pts_all), np.concatenate(tris_all)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr([3] * len(tris))
    mesh.CreateFaceVertexIndicesAttr(tris.reshape(-1).astype(int).tolist())
    mesh.MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set("none")  # static triangle mesh
    return len(pts), len(tris)


def add_box(stage, path, size, xform):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = cube.AddTransformOp()
    xf.Set(Gf.Matrix4d().SetScale(Gf.Vec3d(*size)) * xform)
    cube.MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def safe(name):
    s = re.sub(r"[^A-Za-z0-9_]", "_", name)
    return s if s[0].isalpha() else "_" + s


world_name = os.path.splitext(os.path.basename(args.world))[0]
out_usd = os.path.join(args.out, world_name + ".usd")
os.makedirs(args.out, exist_ok=True)
stage = Usd.Stage.CreateNew(out_usd)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
root = UsdGeom.Xform.Define(stage, "/World").GetPrim()
stage.SetDefaultPrim(root)

skip = re.compile(args.skip, re.I)
collide_roots = []
n_inc = n_skip = 0
for inc in ET.parse(args.world).getroot().iter("include"):
    uri = (inc.findtext("uri") or "").strip()
    name = (inc.findtext("name") or uri.rsplit("/", 1)[-1]).strip()
    model_name = uri.rsplit("/", 1)[-1]
    if skip.search(model_name):
        n_skip += 1
        continue
    mdir = find_model_dir(uri)
    if mdir is None:
        print(f"[WARN] not in fuel cache: {uri}", flush=True)
        continue
    model = ET.parse(os.path.join(mdir, "model.sdf")).getroot().find("model")
    T_model = pose_to_matrix(inc.findtext("pose"))
    base = f"/World/{safe(name)}"
    stage.DefinePrim(base, "Xform")
    for link in model.iter("link"):
        T_link = pose_to_matrix(link.findtext("pose")) * T_model
        meshes = {}  # mesh file -> [scale, xform, visual?, collision?]
        for kind in ("visual", "collision"):
            for el in link.findall(kind):
                T = pose_to_matrix(el.findtext("pose")) * T_link
                geom = el.find("geometry")
                mesh, box = geom.find("mesh"), geom.find("box")
                if mesh is not None:
                    f = resolve_mesh(mesh.findtext("uri").strip(), mdir)
                    scale = [float(s) for s in (mesh.findtext("scale") or "1 1 1").split()]
                    e = meshes.setdefault((f, tuple(scale), str(T)), [scale, T, False, False])
                    e[2 if kind == "visual" else 3] = True
                elif box is not None and kind == "collision":
                    size = [float(s) for s in box.findtext("size").split()]
                    add_box(stage, f"{base}/{safe(link.get('name'))}_{safe(el.get('name'))}", size, T)
        for i, ((f, _, _), (scale, T, vis, col)) in enumerate(meshes.items()):
            usd = convert_mesh(f)
            if usd:
                u = dae_unit(f)
                add_mesh(stage, f"{base}/{safe(link.get('name'))}_m{i}", os.path.relpath(usd, args.out), T,
                         [c * u for c in scale], col, vis)
    n_inc += 1

# merged collider first under /World so RayCaster's "first matching child Mesh" lookup finds it
nv, nt = merge_colliders(stage, collide_roots, "/World/collision")
stage.GetRootLayer().Save()
print(f"[DONE] {out_usd}: {n_inc} models, {n_skip} skipped, {len(_converted)} meshes, collider {nv} verts {nt} tris",
      flush=True)
app.close()
