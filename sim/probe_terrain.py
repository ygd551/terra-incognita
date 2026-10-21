"""Print the floor height map (and ceiling) of a converted scene around a point, from the merged collider.
  ./isaaclab.sh -p /workspace/isaac_explore/sim/probe_terrain.py X Y [half_size=2] [step=0.25] [--scene path.usd]
Use it to see why the robot gets stuck somewhere (steps, holes, low ceilings)."""
import argparse

from isaaclab.app import AppLauncher

p = argparse.ArgumentParser()
p.add_argument("x", type=float)
p.add_argument("y", type=float)
p.add_argument("half", type=float, nargs="?", default=2.0)
p.add_argument("step", type=float, nargs="?", default=0.25)
p.add_argument("--scene", default="/workspace/isaac_explore/sim/scenes/simple_cave_01.usd")
args = p.parse_args()
app = AppLauncher(headless=True).app

import numpy as np  # noqa: E402
import torch  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

from isaaclab.utils.warp import convert_to_warp_mesh, raycast_mesh  # noqa: E402

stage = Usd.Stage.Open(args.scene)
prim = stage.GetPrimAtPath("/World/collision")
if not prim.IsValid():
    raise SystemExit(f"[probe] {args.scene}: no /World/collision (children of /World: "
                     f"{[c.GetName() for c in stage.GetPrimAtPath('/World').GetChildren()][-5:]}); "
                     "re-run sdf_to_usd.py to bake the merged collider")
m = UsdGeom.Mesh(prim)
mesh = convert_to_warp_mesh(np.asarray(m.GetPointsAttr().Get()), np.asarray(m.GetFaceVertexIndicesAttr().Get()),
                            device="cuda")


def cast(starts, dirs):
    o = torch.tensor(starts, dtype=torch.float32, device="cuda")[None]
    d = torch.tensor(dirs, dtype=torch.float32, device="cuda")[None]
    return raycast_mesh(o, d, mesh=mesh, max_dist=60.0)[0][0].cpu().numpy()


xs = np.arange(args.x - args.half, args.x + args.half + 1e-6, args.step)
ys = np.arange(args.y + args.half, args.y - args.half - 1e-6, -args.step)  # print north (+y) at the top
# floor: cast down from 1.0 m above the robot-height plane so ceilings above do not get in the way
z0 = cast([[args.x, args.y, 30.0]], [[0, 0, -1]])[0][2]
floor = np.array([[cast([[x, y, 1.0]], [[0, 0, -1]])[0][2] for x in xs] for y in ys])
print(f"[probe] floor z around ({args.x},{args.y}), rows = y from {ys[0]:.2f} down to {ys[-1]:.2f}, "
      f"cols = x from {xs[0]:.2f} to {xs[-1]:.2f}, step {args.step} m ('  inf' = no floor within 61 m below)", flush=True)
print("        " + " ".join(f"{x:6.2f}" for x in xs), flush=True)
for y, row in zip(ys, floor):
    print(f"{y:6.2f}  " + " ".join(f"{h:6.2f}" for h in row), flush=True)
up = cast([[args.x, args.y, 0.3]], [[0, 0, 1]])[0][2]
print(f"[probe] ceiling above ({args.x},{args.y}): z = {up:.2f}", flush=True)
app.close()
