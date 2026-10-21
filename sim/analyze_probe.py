"""Summarise explore_probe.py snapshots: per snapshot, what TARE sees beyond a split line (e.g. the tunnel mouth).
  python3 analyze_probe.py PROBE_DIR [--split-x 14] [--plot t1,t2,...] [--traj run.csv]
Prints a table (pose, way point, uncovered / frontier / viewpoint candidates / exploring subspaces / terrain_map_ext
points with x > split) and, with --plot, writes PROBE_DIR/snap_<t>.png top views for the chosen sim times."""
import argparse
import csv
import glob
import os

import numpy as np

p = argparse.ArgumentParser()
p.add_argument("dir")
p.add_argument("--split-x", type=float, default=14.0)
p.add_argument("--plot", default="")
p.add_argument("--traj", default="")
p.add_argument("--out", default="", help="where to write PNGs (default: PROBE_DIR)")
p.add_argument("--spawn", default="2,0", help="world x,y of the map origin (go2_sim --spawn), to put GT in map frame")
a = p.parse_args()
snaps = [(float(np.load(f)["t"]), f) for f in sorted(glob.glob(os.path.join(a.dir, "snap_*.npz")))]
X = a.split_x


def beyond(d, k):
    v = d[k] if k in d.files else np.zeros((0, 3))
    return int((v[:, 0] > X).sum()), len(v)


print(f"{'t':>6} {'pose':>17} {'waypoint':>17} | beyond x>{X:g} / total: {'uncov':>11} {'front':>9} "
      f"{'cand':>11} {'subsp':>7} {'terr_ext':>13} {'terr maxx':>9}")
for t, f in snaps:
    d = np.load(f)
    pose = d["pose"]
    wp = d["waypoint"] if "waypoint" in d.files else np.full(3, np.nan)
    te = d["terrain_ext"] if "terrain_ext" in d.files else np.zeros((0, 4))
    cols = [beyond(d, k) for k in ("uncovered", "frontier", "candidates", "subspaces", "terrain_ext")]
    extra = " ".join(f"{k}={beyond(d, k)[0]}/{beyond(d, k)[1]}" for k in ("frontier_raw", "frontier_clustered")
                     if k in d.files)
    if "cells" in d.files and len(d["cells"]):
        c = d["cells"][d["cells"][:, 0] > X]
        extra += " cells>X: " + " ".join(f"{n}={int((c[:, 3] == v).sum())}" for v, n in ((1, "EXPL"), (2, "COV"),
                                                                                    (3, "COV_OTH"), (4, "NOGO")))
    print(f"{t:6.0f} ({pose[0]:5.1f},{pose[1]:5.1f},{pose[2]:4.1f}) ({wp[0]:5.1f},{wp[1]:5.1f},{wp[2]:4.1f}) | "
          + " ".join(f"{b:5d}/{n:<5d}" for b, n in cols[:3]) + f" {cols[3][0]:3d}/{cols[3][1]:<3d}"
          + f" {cols[4][0]:6d}/{cols[4][1]:<6d} {te[:, 0].max() if len(te) else np.nan:9.1f} {extra}")

if a.plot:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    gt = None
    if a.traj:
        rows = [r for r in csv.DictReader(open(a.traj)) if r["topic"] == "/ground_truth/odom"]
        gt = np.array([[float(r["t"]), float(r["x"]), float(r["y"])] for r in rows])
        gt[:, 1:] -= [float(v) for v in a.spawn.split(",")]
    for want in (float(s) for s in a.plot.split(",")):
        t, f = min(snaps, key=lambda s: abs(s[0] - want))
        d = np.load(f)
        fig, ax = plt.subplots(figsize=(12, 9))
        if "occupancy" in d.files and len(d["occupancy"]):
            oc = d["occupancy"]
            for v, col, lab in ((1, "#cfe8ff", "TARE occupancy: free"), (0, "#555555", "TARE occupancy: occupied")):
                m = oc[:, 3] == v
                ax.scatter(oc[m, 0], oc[m, 1], c=col, s=2, marker="s", label=lab, zorder=0)
        if "cells" in d.files and len(d["cells"]):
            from matplotlib.patches import Rectangle
            for cx, cy, _, st in d["cells"]:
                col = {1: "tab:blue", 2: "tab:gray", 3: "tab:olive", 4: "tab:red"}.get(int(st), "k")
                ax.add_patch(Rectangle((cx - 3, cy - 3), 6, 6, fill=False, ec=col, lw=1.5 if st == 1 else 0.8,
                                       ls="-" if st == 1 else "--", zorder=1))
                ax.text(cx, cy + 2.3, {1: "EXPL", 2: "COV", 3: "COV_OTH", 4: "NOGO"}.get(int(st), "?"), color=col,
                        fontsize=6, ha="center", zorder=1)
        if "terrain_ext" in d.files and len(d["terrain_ext"]):
            te = d["terrain_ext"]
            sc = ax.scatter(te[:, 0], te[:, 1], c=np.clip(te[:, 3], 0, 0.6), s=1, cmap="viridis", alpha=0.5)
            fig.colorbar(sc, ax=ax, fraction=0.03, label="terrain_map_ext intensity (height above ground)")
        for k, style in (("uncovered", dict(c="orange", s=4, label="uncovered surface")),
                         ("frontier_raw", dict(c="pink", s=5, label="raw frontier (occupancy)")),
                         ("frontier_clustered", dict(c="purple", s=6, label="frontier after filter+cluster")),
                         ("frontier", dict(c="red", s=8, label="uncovered frontier")),
                         ("candidates", dict(c="k", s=6, marker="+", label="viewpoint candidates")),
                         ("selected", dict(c="m", s=40, marker="*", label="selected viewpoints")),
                         ("subspaces", dict(c="b", s=60, marker="s", facecolors="none", label="exploring subspaces"))):
            if k in d.files and len(d[k]):
                ax.scatter(d[k][:, 0], d[k][:, 1], **style)
        for k, style in (("local_path", "g-"), ("global_path", "b--")):
            if k in d.files and len(d[k]):
                ax.plot(d[k][:, 0], d[k][:, 1], style, lw=1.5, label=k)
        if gt is not None:
            g = gt[gt[:, 0] <= t]
            ax.plot(g[:, 1], g[:, 2], "c-", lw=1, label="ground truth so far")
        ax.plot(*d["pose"][:2], "ro", ms=8, label="robot (map)")
        if "waypoint" in d.files:
            ax.plot(*d["waypoint"][:2], "g^", ms=10, label="way point")
        ax.axvline(X, color="gray", ls=":")
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8)
        ax.set_title(f"t = {t:.0f} s (sim)")
        out = os.path.join(a.out or a.dir, f"snap_{t:05.0f}.png")
        fig.savefig(out, dpi=100, bbox_inches="tight")
        plt.close(fig)
        print("wrote", out)
