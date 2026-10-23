#!/usr/bin/env python3
"""Exploration progress of one run in a SubT world: tiles entered, distance and explored volume over sim time.

  python3 tile_coverage.py logs/trial_X/run.csv [logs/trial_X/metrics.txt] [--world NAME] [--step 300] [--until 1800]

run.csv = sim/record_run.py output (/ground_truth/odom rows), or a visualizationTools trajectory_*.txt
(x y z roll pitch yaw t). metrics.txt = visualizationTools metrics (volume distance runtime t), optional.
--world: assets/subt/worlds/NAME.sdf (default: the scene ./sim.sh last started, logs/.scene, else simple_cave_01).
A tile counts as entered once the robot is within its cell (half the tile size, from the .dot generator's --scale_x:
|dx|, |dy| < 12.5 m for 25 m cave tiles) and 6 m of its height, the same rule as the TARE baseline in simple_cave_01
(18 / 51 tiles over a ~3000 s run)."""
import argparse
import os
import re

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../..")


def world_files(name):
    """(sdf path, half the tile size [m]) of a SubT world."""
    sdf = os.path.join(ROOT, "assets/subt/worlds", name + ".sdf")
    dot = sdf[:-4] + ".dot"
    m = re.search(r"scale_x (\d+(?:\.\d+)?)", open(dot).read()) if os.path.exists(dot) else None
    return sdf, float(m.group(1)) / 2 if m else 12.5


def load_tiles(sdf):
    """Poses of the course pieces: the includes that are vertices of the world's .dot topology graph (label
    "id::model::instance", matched by instance name, e.g. tile_12, or by model: niosh_ex_config_* names its sections
    differently in the .dot and the .sdf), else the tile_* includes.
    ponytail: NIOSH sections are not 25 m grid tiles, there the cell test is only approximate."""
    dot = sdf[:-4] + ".dot"
    labels = re.findall(r'label="\d+::([^:"]*)::([^"]+)"', open(dot).read()) if os.path.exists(dot) else []
    labels = [(m, i) for m, i in labels if i != "BaseStation"]
    models, names = {m for m, _ in labels}, {i for _, i in labels}
    tiles = []
    for b in re.findall(r"<include>(.*?)</include>", open(sdf).read(), re.S):
        n, p = re.search(r"<name>(.*?)</name>", b), re.search(r"<pose>(.*?)</pose>", b)
        u = re.search(r"models/([^<]*?)\s*</uri>", b)
        piece = (n.group(1) in names or (u and u.group(1) in models)) if labels else n and n.group(1).startswith("tile_")
        if n and p and piece:
            tiles.append([float(v) for v in p.group(1).split()[:3]])
    return np.array(tiles)


def load_track(path):
    """-> (N, 4) array of t, x, y, z."""
    if path.endswith(".csv"):
        rows = [r.split(",") for r in open(path)][1:]
        return np.array([[float(r[0]), float(r[2]), float(r[3]), float(r[4])] for r in rows
                         if len(r) > 4 and r[1] == "/ground_truth/odom" and r[2]])
    rows = [r.split() for r in open(path)]
    return np.array([[float(r[6]), float(r[0]), float(r[1]), float(r[2])] for r in rows if len(r) == 7])


def first_entry(track, tiles, half=12.5):
    """Sim time each tile was first entered (inf if never)."""
    d = np.abs(track[:, None, 1:] - tiles[None])                                  # (N, T, 3)
    inside = (d[..., 0] < half) & (d[..., 1] < half) & (d[..., 2] < 6.0)
    return np.where(inside.any(0), track[inside.argmax(0), 0], np.inf)


def selftest():
    tiles = np.array([[25.0, 0, 0], [50, 0, 0], [50, 0, 20]])
    track = np.array([[0.0, 2, 0, 0.5], [10, 20, 0, 0.5], [20, 40, 1, 0.5]])
    assert first_entry(track, tiles).tolist() == [10.0, 20.0, np.inf]
    print("selftest ok")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("track", nargs="?")
    ap.add_argument("metrics", nargs="?")
    ap.add_argument("--world", default=None)
    ap.add_argument("--step", type=float, default=300.0)
    ap.add_argument("--until", type=float, default=None)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    scene = os.path.join(ROOT, "logs/.scene")
    world = a.world or (open(scene).read().strip() if os.path.exists(scene) else "simple_cave_01")
    sdf, half = world_files(world)
    tiles, track = load_tiles(sdf), load_track(a.track)
    t0 = track[0, 0]
    end = a.until or track[-1, 0] - t0
    track = track[track[:, 0] - t0 <= end]
    entered = first_entry(track, tiles, half) - t0
    seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(track[:, 1:3], axis=0), axis=1))]
    # volume, t (complete lines only: the file of a running stack ends mid-line)
    met = np.array([[r[0], r[3]] for r in map(str.split, open(a.metrics)) if len(r) == 4], float) if a.metrics else None
    print(f"{a.track}: {track[-1, 0] - t0:.0f} s, end at ({track[-1, 1]:.1f}, {track[-1, 2]:.1f}, {track[-1, 3]:.1f})")
    print("    t [s]  tiles  dist [m]  volume [m3]")
    for t in [*np.arange(a.step, end, a.step), end]:
        k = np.searchsorted(track[:, 0] - t0, t, side="right") - 1
        vol = ""
        if met is not None and len(met):
            j = np.searchsorted(met[:, 1], t0 + t, side="right") - 1   # both stamped in sim time
            vol = f"{met[max(j, 0), 0]:11.0f}"
        print(f"  {t:7.0f}  {int((entered <= t).sum()):2d}/{len(tiles)}  {seg[k]:8.0f}  {vol}")


if __name__ == "__main__":
    main()
