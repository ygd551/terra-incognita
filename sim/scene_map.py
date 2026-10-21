"""Top-down floor / clearance map of a converted scene (from the merged collider), optionally with a trajectory.
  ./isaaclab.sh -p /workspace/isaac_explore/sim/scene_map.py X0 X1 Y0 Y1 [--step 0.2] [--z 0.5] [--scene ...]
      [--traj logs/.../run.csv] [--out map.png]
From a horizontal plane at height --z, casts one ray down (floor) and one up (ceiling) per cell. Writes a PNG with
floor height and clearance (ceiling - floor; dark = rock / too low for the Go2) and prints an ASCII clearance map."""
import argparse

from isaaclab.app import AppLauncher

p = argparse.ArgumentParser()
p.add_argument("bounds", type=float, nargs=4, metavar=("X0", "X1", "Y0", "Y1"))
p.add_argument("--step", type=float, default=0.2)
p.add_argument("--z", type=float, default=0.5)
p.add_argument("--scene", default="/workspace/isaac_explore/sim/scenes/simple_cave_01.usd")
p.add_argument("--traj", default="", help="run.csv from record_run.py: ground-truth path is overlaid")
p.add_argument("--out", default="/workspace/isaac_explore/logs/scene_map.png")
args = p.parse_args()
app = AppLauncher(headless=True).app

import csv  # noqa: E402

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

from isaaclab.utils.warp import convert_to_warp_mesh, raycast_mesh  # noqa: E402

stage = Usd.Stage.Open(args.scene)
prim = stage.GetPrimAtPath("/World/collision")
if not prim.IsValid():  # older conversions: take the largest mesh named "collision"
    cands = [q for q in stage.Traverse() if q.IsA(UsdGeom.Mesh) and q.GetName() == "collision"]
    print(f"[map] no /World/collision; meshes named collision: {[str(q.GetPath()) for q in cands][:5]}")
    prim = max(cands, key=lambda q: len(UsdGeom.Mesh(q).GetPointsAttr().Get() or []))
m = UsdGeom.Mesh(prim)
pts = np.asarray(m.GetPointsAttr().Get())
xf = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T  # row-major -> col
pts = pts @ xf[:3, :3].T + xf[:3, 3]
print(f"[map] mesh {prim.GetPath()}: {len(pts)} vertices, bbox {pts.min(0).round(1)} .. {pts.max(0).round(1)}")
mesh = convert_to_warp_mesh(pts, np.asarray(m.GetFaceVertexIndicesAttr().Get()), device="cuda")
x0, x1, y0, y1 = args.bounds
xs, ys = np.arange(x0, x1, args.step), np.arange(y0, y1, args.step)
X, Y = np.meshgrid(xs, ys)
starts = torch.tensor(np.stack([X.ravel(), Y.ravel(), np.full(X.size, args.z)], 1), dtype=torch.float32,
                      device="cuda")[None]


def cast(dz):
    d = torch.zeros_like(starts)
    d[..., 2] = dz
    return raycast_mesh(starts, d, mesh=mesh, max_dist=40.0)[0][0, :, 2].cpu().numpy().reshape(X.shape)


floor, ceil = cast(-1.0), cast(1.0)
clear = ceil - floor  # inf where a ray escaped
print(f"[map] {len(xs)} x {len(ys)} cells, plane z = {args.z}")
# ASCII: rows = y (north on top), one char per 1 m: '#' rock/no gap (<0.3 m), '-' low (<0.6 m), '.' free, ' ' no floor
k = max(1, round(1.0 / args.step))
print("[map] clearance, 1 char = 1 m: '#' < 0.3 m  '-' < 0.6 m  '.' free  ' ' open (no floor/ceiling)")
for j in range(len(ys) - 1, -1, -k):
    row = ""
    for i in range(0, len(xs), k):
        c = clear[j, i]
        row += " " if not np.isfinite(c) else "#" if c < 0.3 else "-" if c < 0.6 else "."
    print(f"{ys[j]:7.1f} {row}")
print("        " + "".join(str(int(abs(x)) // 10 % 10) if int(x) % 10 == 0 else " " for x in xs[::k]))

fig, ax = plt.subplots(1, 2, figsize=(16, 8 * (y1 - y0) / (x1 - x0) + 1.5), sharey=True)
ext = (x0, x1, y0, y1)
f = np.where(np.isfinite(floor) & (clear > 0.3), floor, np.nan)
im0 = ax[0].imshow(f, origin="lower", extent=ext, cmap="terrain")
ax[0].set_title(f"floor z (where clearance > 0.3 m), plane z={args.z}")
fig.colorbar(im0, ax=ax[0], fraction=0.03)
im1 = ax[1].imshow(np.clip(np.where(np.isfinite(clear), clear, 5.0), 0, 5), origin="lower", extent=ext, cmap="magma")
ax[1].set_title("clearance ceiling - floor [m] (dark = rock / low)")
fig.colorbar(im1, ax=ax[1], fraction=0.03)
if args.traj:
    rows = [r for r in csv.DictReader(open(args.traj)) if r["topic"] == "/ground_truth/odom"]
    gx, gy = [float(r["x"]) for r in rows], [float(r["y"]) for r in rows]
    for a in ax:
        a.plot(gx, gy, "c-", lw=1.2)
        a.plot(gx[:1], gy[:1], "go")
        a.plot(gx[-1:], gy[-1:], "rx")
for a in ax:
    a.set_xlabel("x [m]")
    a.grid(alpha=0.3)
ax[0].set_ylabel("y [m]")
fig.tight_layout()
fig.savefig(args.out, dpi=110)
print(f"[map] wrote {args.out}")
app.close()
