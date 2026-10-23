"""Find a start position in a converted scene: the free floor point nearest to (x, y), i.e. a collider surface with at
least HEADROOM m free above it (the robot fits), and its height - the [x, y, z_ref] for config/scenes.yaml.
  /isaac-sim/python.sh tools/scene_spawn.py NAME [X Y]    (inside the isaac_explore container; default X Y = 0 0)"""
import sys

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom  # noqa: E402

HEADROOM, STEP, RADIUS = 1.5, 1.0, 30.0


def surfaces(a, b, c, x, y):
    """Heights of all triangles above / below (x, y), ascending."""
    e1, e2, q = b[:, :2] - a[:, :2], c[:, :2] - a[:, :2], np.array([x, y]) - a[:, :2]
    den = e1[:, 0] * e2[:, 1] - e2[:, 0] * e1[:, 1]
    den = np.where(np.abs(den) > 1e-12, den, np.nan)
    u = (q[:, 0] * e2[:, 1] - e2[:, 0] * q[:, 1]) / den
    v = (e1[:, 0] * q[:, 1] - q[:, 0] * e1[:, 1]) / den
    hit = (u >= 0) & (v >= 0) & (u + v <= 1)
    z = np.sort((a[:, 2] + u * (b[:, 2] - a[:, 2]) + v * (c[:, 2] - a[:, 2]))[hit])
    return z[np.r_[True, np.diff(z) > 0.01]] if len(z) else z   # pittsburgh_mine has every triangle twice


name = sys.argv[1]
x0, y0 = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (0.0, 0.0)
stage = Usd.Stage.Open(f"/workspace/isaac_explore/sim/scenes/{name}.usd")
mesh = UsdGeom.Mesh(stage.GetPrimAtPath("/World/collision"))   # the merged collider (sim/sdf_to_usd.py)
p = np.array(mesh.GetPointsAttr().Get(), float)
tri = np.array(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
print(f"{name}: {len(tri)} triangles, x {p[:, 0].min():.0f}..{p[:, 0].max():.0f} y {p[:, 1].min():.0f}..{p[:, 1].max():.0f} "
      f"z {p[:, 2].min():.1f}..{p[:, 2].max():.1f}", flush=True)
best = None
for r in np.arange(0.0, RADIUS + 1e-9, STEP):        # rings of growing radius around (x0, y0)
    ring = [(x0, y0)] if r == 0 else [(x0 + r * np.cos(t), y0 + r * np.sin(t))
                                      for t in np.linspace(0, 2 * np.pi, max(8, int(2 * np.pi * r / STEP)), False)]
    for x, y in ring:
        near = np.where((np.abs(p[tri, 0] - x).min(1) < 50) & (np.abs(p[tri, 1] - y).min(1) < 50))[0]
        a, b, c = (p[i] for i in tri[near].T)
        z = surfaces(a, b, c, x, y)
        for k in range(len(z)):          # a floor with HEADROOM free above it (next surface up, or open sky)
            if (k == len(z) - 1 or z[k + 1] - z[k] > HEADROOM) and (k == 0 or z[k] - z[k - 1] > 0.05):
                best = (x, y, z[k], z.tolist())
                break
        if best:
            break
    if best:
        break
print("spawn: [%.1f, %.1f, %.1f]   surfaces there: %s" % (best[0], best[1], best[2], [round(v, 2) for v in best[3]])
      if best else "no free floor within %.0f m" % RADIUS, flush=True)
app.close()
