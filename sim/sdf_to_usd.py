"""Convert a Gazebo/Ignition SDF world (DARPA SubT tiles) into a USD stage for Isaac Sim.

Run inside the isaac_explore container (setup.sh does it for simple_cave_01; tools/fetch_assets.py downloads the world
and the Fuel models it uses into assets/):
  /isaac-sim/python.sh /workspace/isaac_explore/sim/sdf_to_usd.py /workspace/isaac_explore/assets/subt/worlds/simple_cave_01.sdf
Output: sim/scenes/<world>.usd, meshes cached in sim/scenes/meshes/ (converted once, reused by all worlds), and
sim/scenes/<world>.artifacts.yaml: the SubT artifacts (survivors, phones, backpacks, ...) of the world with their
true positions (world frame), for scoring what the robot reports.

Only <include> models are handled (that is all SubT worlds use; also those inside a model, which the tiles with lamps
use), artifacts and the base station included, as in the
competition. Each model's visual and collision meshes are referenced at their composed SDF pose and baked into two
meshes, as Gazebo uses them: /World/collision (all collision geometry: the static triangle collider, which the height
scanner also ray-casts) and /World/lidar (all visual geometry: what Gazebo's gpu_lidar renders, so the drawn fences of
the staging areas and the detailed rock of the rough tunnel tiles, not the invisible walls or the simplified colliders).
Models that events deploy later (the debris walls that close a route once a robot has passed) are left out.
"""
import argparse
import asyncio
import glob
import math
import os
import re
import sys
import urllib.parse
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser()
parser.add_argument("world", help="path to .sdf world")
parser.add_argument("--fuel", default="/workspace/isaac_explore/assets/fuel", help="ignition fuel cache root")
parser.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenes"))
parser.add_argument("--skip", help="regex (case-insensitive) of model names to leave out, e.g. 'randy|phone'")
args = parser.parse_args()
# SubT artifact models (Fuel names): their <include> name minus the number is the artifact type ("rescue_randy_1")
ARTIFACT = re.compile("backpack|randy|phone|samsung|helmet|rope|vent|gas|drill|extinguisher|cube", re.I)

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
    """Fuel URI -> newest cached version dir, e.g. /fuel/fuel.ignitionrobotics.org/openrobotics/models/<name>/<ver>.
    model://<name> (a Gazebo-classic local model) -> a model of that name under any host / owner of the cache, e.g.
    the gbplanner mine scans copied to <fuel>/local/gbplanner/models/<name>/1."""
    m = re.search(r"https?://([^/]+)/[^/]+/([^/]+)/models/([^/]+)", uri)
    if m:
        cands = [os.path.join(args.fuel, host, m.group(2).lower(), "models", m.group(3).lower())
                 for host in (m.group(1), "fuel.ignitionrobotics.org", "fuel.gazebosim.org")]
    elif uri.startswith("model://"):
        cands = glob.glob(os.path.join(args.fuel, "*", "*", "models", uri[8:].split("/")[0].lower()))
    else:
        return None
    for d in cands:
        if os.path.isdir(d):
            vers = sorted((v for v in os.listdir(d) if v.isdigit()), key=int)
            if vers:
                return os.path.join(d, vers[-1])
    return None


def resolve_mesh(uri, mdir):
    """Mesh (or texture) URI -> local file. Either relative to the model dir, or a full fuel URL
    '.../models/<name>/<version or tip>/files/<path>' pointing into another (or the same) cached model."""
    uri = urllib.parse.unquote(uri)
    m = re.match(r"https?://.+?/models/[^/]+/(?:\d+|tip)/files/(.+)", uri)
    if m:
        return os.path.join(find_model_dir(uri) or mdir, m.group(1))
    return os.path.join(mdir, uri.replace("model://", "").split("/", 1)[-1] if uri.startswith("model://") else uri)


_converted = {}
FUEL_FILE = re.compile(r"https?://[^<>\"]+?/models/[^/<>\"]+/(?:\d+|tip)/files/[^<>\"]+")


def convert_mesh(src):
    """DAE/OBJ/STL -> USD (cached on disk). Returns usd path or None."""
    if src in _converted:
        return _converted[src]
    rel = os.path.relpath(src, args.fuel)
    dst = os.path.join(args.out, "meshes", os.path.splitext(rel)[0] + ".usd")
    # most SubT tiles name their textures by Fuel URL: Gazebo maps that to its cache, the asset converter cannot (the
    # tile rendered plain grey), so convert a copy that names the downloaded files instead; a cached conversion without
    # that copy next to it is from before (v3.0.0) and is converted again
    text = open(src, errors="ignore").read() if src.lower().endswith(".dae") else ""
    local = FUEL_FILE.sub(lambda m: resolve_mesh(m.group(0), os.path.dirname(src)), text)
    inp = dst[:-4] + ".dae" if local != text else src
    if not os.path.exists(dst) or not os.path.exists(inp):
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        if inp != src:
            with open(inp, "w") as fh:
                fh.write(local)
        ctx = conv.AssetConverterContext()
        ctx.use_meter_as_world_unit = True  # SDF is in meters
        ctx.ignore_materials = False
        ctx.ignore_animations = True
        ctx.ignore_camera = True
        ctx.ignore_light = True
        task = conv.get_instance().create_converter_task(inp, dst, None, ctx)
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
    if visible:
        visual_roots.append(prim)
    else:
        UsdGeom.Imageable(prim).MakeInvisible()
    if collide:
        collide_roots.append(prim)


def merge_meshes(stage, roots, path, collider):
    """Bake every mesh under roots (world frame, triangulated) into ONE invisible triangle mesh, a static triangle
    collider if collider. Isaac Lab's RayCaster (the height scan of the walking policy, the lidar) only supports a single
    mesh prim, and one big static collider is also cheaper for PhysX than hundreds of referenced ones."""
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
    if collider:
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set("none")  # static triangle mesh
    return len(tris)


def add_box(stage, path, size, xform):
    """Box collider, as an invisible box mesh that merge_meshes bakes into the collider, which the height scanner also
    ray-casts (as a separate physics-only Cube, the 1 mm floors of the tunnel tiles made the height scan see a cliff and
    the robot toppled). Being collision geometry, it is not in the lidar mesh: like in Gazebo, the lidar does not see
    the invisible enclosure walls and roofs of the staging areas, only their drawn fences and rock walls."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(x, y, z) for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)]))
    mesh.CreateFaceVertexCountsAttr([4] * 6)   # corner i = 4x + 2y + z (bits)
    mesh.CreateFaceVertexIndicesAttr([0, 1, 3, 2, 4, 6, 7, 5, 0, 4, 5, 1, 2, 3, 7, 6, 0, 2, 6, 4, 1, 5, 7, 3])
    mesh.AddTransformOp().Set(Gf.Matrix4d().SetScale(Gf.Vec3d(*size)) * xform)
    mesh.MakeInvisible()
    collide_roots.append(mesh.GetPrim())


def safe(name):
    s = re.sub(r"[^A-Za-z0-9_]", "_", name)
    return s if s[0].isalpha() else "_" + s


def add_model(uri, T_model, base):
    """A model's links (meshes, boxes) under the prim base at T_model, and the models that it includes itself (the
    SubT tiles with lamps, "... Lights", include the tile without them and add the lamps). Not the includes inside its
    plugins: events deploy those (the rocks of a rock fall). False if the model is not in the Fuel cache."""
    mdir = find_model_dir(uri)
    if mdir is None:
        print(f"[WARN] not in fuel cache: {uri}", flush=True)
        return False
    # lstrip: some Fuel models (Phone, Valve, Backpack, ...) start with a blank line before <?xml>, which ET rejects
    model = ET.fromstring(open(os.path.join(mdir, "model.sdf"), encoding="utf-8").read().lstrip()).find("model")
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
    for sub in model.findall("include"):
        add_model((sub.findtext("uri") or "").strip(), pose_to_matrix(sub.findtext("pose")) * T_model,
                  f"{base}/{safe((sub.findtext('name') or 'include').strip())}")
    return True


world_name = os.path.splitext(os.path.basename(args.world))[0]
out_usd = os.path.join(args.out, world_name + ".usd")
os.makedirs(args.out, exist_ok=True)
stage = Usd.Stage.CreateNew(out_usd)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
root = UsdGeom.Xform.Define(stage, "/World").GetPrim()
stage.SetDefaultPrim(root)

skip = re.compile(args.skip, re.I) if args.skip else None
collide_roots, visual_roots, artifacts = [], [], []
n_inc = n_skip = 0
world = ET.parse(args.world).getroot()
deployed = {i for p in world.iter("plugin") for i in p.iter("include")}  # by events (Breadcrumbs plugin), not at start
for inc in world.iter("include"):
    uri = (inc.findtext("uri") or "").strip()
    name = (inc.findtext("name") or uri.rsplit("/", 1)[-1]).strip()
    model_name = uri.rsplit("/", 1)[-1]
    if inc in deployed or (skip and skip.search(model_name)):
        n_skip += 1
        continue
    if ARTIFACT.search(model_name):
        artifacts.append({"name": name, "type": re.sub(r"_\d+$", "", name), "model": model_name,
                          "pos": [round(float(c), 3) for c in (inc.findtext("pose") or "0 0 0").split()[:3]]})
    if add_model(uri, pose_to_matrix(inc.findtext("pose")), f"/World/{safe(name)}"):
        n_inc += 1

nt = merge_meshes(stage, collide_roots, "/World/collision", collider=True)
nl = merge_meshes(stage, visual_roots, "/World/lidar", collider=False)
stage.GetRootLayer().Save()
with open(os.path.join(args.out, world_name + ".artifacts.yaml"), "w") as f:
    import yaml
    f.write(f"# SubT artifacts of {os.path.basename(args.world)}: true positions (world frame), by sim/sdf_to_usd.py\n")
    yaml.safe_dump(artifacts, f, default_flow_style=None, sort_keys=False)
print(f"[DONE] {out_usd}: {n_inc} models, {n_skip} skipped, {len(artifacts)} artifacts, {len(_converted)} meshes, "
      f"collider {nt} tris, lidar mesh {nl} tris", flush=True)
app.close()
