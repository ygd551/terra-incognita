#!/usr/bin/env python3
"""How smoothly the robot followed GBPlanner2's paths, from a ros1_ws/scripts/record_tracking.py recording:
  python3 tools/tracking_report.py ros1_ws/logs/tracking.json      -> numbers + <recording>.png next to it
Numbers (weighted by path length): turning per metre of the planned paths and of the flown trajectory (both resampled
every 0.5 m: a flown value far above the planned one = weaving), the distance from the path, and for drones the speed
against the commanded one, the commanded yaw rate (share of time at the 0.5 rad/s limit) and the nose against the
direction of motion. Plot: paths and trajectory from above; commanded/flown direction, yaw rate and speed over time."""
import json
import sys

import numpy as np


def resample(P, step=0.5):
    P = np.asarray(P, float)
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    if s[-1] < step:
        return P[[0, -1]]
    q = np.arange(0, s[-1], step)
    return np.c_[[np.interp(q, s, P[:, k]) for k in range(P.shape[1])]].T


def turn_per_m(P):
    """Mean |heading change| per metre on a 0.5 m resampling [deg/m] (0 for short pieces)."""
    v = np.diff(resample(P)[:, :2], axis=0)
    v = v[np.linalg.norm(v, axis=1) > 1e-3]
    if len(v) < 2:
        return 0.0
    return np.abs(np.degrees(np.diff(np.unwrap(np.arctan2(v[:, 1], v[:, 0]))))).sum() / (0.5 * len(v))


def dist_to_path(q, P):
    a, b = P[:-1], P[1:]
    ab = b - a
    t = np.clip(np.einsum("ij,ij->i", q - a, ab) / np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-12), 0, 1)
    return np.linalg.norm(a + t[:, None] * ab - q, axis=1).min()


def selftest():
    line = np.c_[np.linspace(0, 10, 50), np.zeros(50), np.zeros(50)]
    assert turn_per_m(line) < 1e-6
    x = np.linspace(0, 10, 400)
    zig = np.c_[x, 0.3 * np.sin(2 * np.pi * x / 2.5), np.zeros_like(x)]   # weaving: 0.3 m amplitude, 2.5 m wavelength
    assert turn_per_m(zig) > 30, turn_per_m(zig)
    assert abs(dist_to_path(np.array([5.0, 0.4, 0.0]), line) - 0.4) < 1e-9
    print("selftest ok")


def main(path):
    d = json.load(open(path))
    O, C, paths = np.array(d["odom"]), np.array(d["cmd"]), d["paths"]
    rows = []
    for i, pa in enumerate(paths):
        P = np.array(pa["pts"])
        t1 = paths[i + 1]["t"] if i + 1 < len(paths) else O[-1, 0]
        seg = O[(O[:, 0] >= pa["t"]) & (O[:, 0] < t1), 1:4]
        if len(P) < 2 or len(seg) < 4:
            continue
        L = np.linalg.norm(np.diff(P, axis=0), axis=1).sum()
        rows.append((L, turn_per_m(P), turn_per_m(seg), np.mean([dist_to_path(q, P) for q in seg])))
    r = np.array(rows)
    w = r[:, 0] / r[:, 0].sum()
    print(f"{len(paths)} paths, {r[:, 0].sum():.0f} m planned")
    print(f"turning per metre: planned {w @ r[:, 1]:.1f} deg/m, flown {w @ r[:, 2]:.1f} deg/m; "
          f"distance from the path: mean {w @ r[:, 3]:.2f} m")
    t, x, y, yaw = O[:, 0], O[:, 1], O[:, 2], np.unwrap(O[:, 4])
    vx, vy = np.gradient(x, t), np.gradient(y, t)
    vdir = np.unwrap(np.arctan2(vy, vx))
    if len(C):
        cyaw = np.interp(C[:, 0], t, yaw)
        cw = np.c_[np.cos(cyaw) * C[:, 1] - np.sin(cyaw) * C[:, 2], np.sin(cyaw) * C[:, 1] + np.cos(cyaw) * C[:, 2]]
        cspd, spd = np.linalg.norm(cw, axis=1), np.interp(C[:, 0], t, np.hypot(vx, vy))
        m = (cspd > 0.5) & (spd > 0.5)
        va = np.interp(C[:, 0], t, vdir)
        err = np.degrees((va - np.arctan2(cw[:, 1], cw[:, 0]) + np.pi) % (2 * np.pi) - np.pi)[m]
        nose = np.degrees((cyaw - va + np.pi) % (2 * np.pi) - np.pi)[m]
        print(f"speed while commanded 1 m/s: mean {spd[cspd > 0.95].mean():.2f}, max {spd.max():.2f} m/s; flown "
              f"direction vs commanded: mean {np.abs(err).mean():.0f} deg; commanded yaw rate at the 0.5 rad/s "
              f"limit {np.mean(np.abs(C[:, 4]) > 0.49):.0%} of the time; nose vs motion: mean {np.abs(nose).mean():.0f} deg")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(15, 7))
    ax = fig.add_subplot(1, 2, 1)
    for i, pa in enumerate(paths):
        P = np.array(pa["pts"])
        ax.plot(P[:, 0], P[:, 1], lw=2.5, alpha=0.6, label="GBPlanner2 paths" if i == 0 else None)
    ax.plot(x, y, "k-", lw=0.8, label="flown")
    ax.set_aspect("equal"), ax.grid(alpha=0.3), ax.legend(), ax.set_xlabel("x [m]"), ax.set_ylabel("y [m]")
    if len(C):
        tt = C[:, 0] - C[0, 0]
        for k, (vals, lab) in enumerate([((np.degrees(np.unwrap(np.arctan2(cw[:, 1], cw[:, 0]))),
                                           np.degrees(np.interp(C[:, 0], t, vdir)), np.degrees(cyaw)),
                                          ("commanded direction", "flown direction", "nose (yaw)")),
                                         ((C[:, 4],), ("commanded yaw rate [rad/s]",)),
                                         ((cspd, spd), ("commanded speed [m/s]", "flown speed [m/s]"))]):
            a = fig.add_subplot(3, 2, 2 * k + 2)
            for v, l in zip(vals, lab):
                a.plot(tt, v, label=l)
            a.grid(alpha=0.3), a.legend(fontsize=8)
        a.set_xlabel("sim time [s]")
    out = path.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=100, bbox_inches="tight")
    print(f"plot: {out}")


if __name__ == "__main__":
    selftest() if sys.argv[1:] == ["--selftest"] else main(sys.argv[1])
