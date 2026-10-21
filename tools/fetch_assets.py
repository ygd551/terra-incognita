#!/usr/bin/env python3
"""Download what the simulation needs into assets/ (called by setup.sh; safe to re-run, finished files are kept).

  assets/checkpoints/<robot>_rough.pt   Isaac Lab's published rough-terrain walking policies (Go2, ANYmal D)
  assets/subt/worlds/<world>.sdf        DARPA SubT world from osrf/subt (pinned commit)
  assets/fuel/fuel.gazebosim.org/openrobotics/models/<name>/<version>/
                                        only the Fuel models the world uses (CC-BY 4.0), in the layout of the Gazebo
                                        fuel cache that sim/sdf_to_usd.py reads

Usage: tools/fetch_assets.py [world ...]      (default: simple_cave_01)
       tools/fetch_assets.py --selftest       (offline check of the URI parsing)
"""
import io
import json
import os
import re
import sys
import time
import urllib.parse
import urllib.request
import zipfile

ASSETS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets")
CHECKPOINT_URL = ("http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/IsaacLab/"
                  "PretrainedCheckpoints/rsl_rl/{}/checkpoint.pt")
CHECKPOINTS = {"go2_rough.pt": "Isaac-Velocity-Rough-Unitree-Go2-v0", "anymal_d_rough.pt": "Isaac-Velocity-Rough-Anymal-D-v0"}
SUBT_URL = "https://raw.githubusercontent.com/osrf/subt/ba15f9ff35ccfb43f44937d6d84e047615d174ff/subt_ign/worlds/{}.sdf"
FUEL = "https://fuel.gazebosim.org/1.0"
# a model reference anywhere in an SDF: <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Cave Cap Type B</uri>
# (also model://-style mesh URIs that point into another fuel model: .../models/<name>/tip/files/meshes/x.dae)
FUEL_REF = re.compile(r"fuel\.(?:ignitionrobotics|gazebosim)\.org/1\.0/([^/<\"]+)/models/([^/<\"]+)")


def get(url, tries=6):
    for i in range(tries):
        try:
            with urllib.request.urlopen(url, timeout=120) as r:
                return r.read()
        except Exception as e:  # flaky links: retry with back-off
            if i == tries - 1:
                raise
            print(f"  retry ({e}): {url}", flush=True)
            time.sleep(3 * (i + 1))


def save(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path + ".part", "wb") as f:
        f.write(data)
    os.replace(path + ".part", path)


def fetch_model(owner, name, done):
    owner, name = urllib.parse.unquote(owner), urllib.parse.unquote(name)
    if (owner.lower(), name.lower()) in done:
        return
    done.add((owner.lower(), name.lower()))
    root = os.path.join(ASSETS, "fuel", "fuel.gazebosim.org", owner.lower(), "models", name.lower())
    versions = sorted((d for d in os.listdir(root) if d.isdigit()), key=int) if os.path.isdir(root) else []
    if versions:
        vdir = os.path.join(root, versions[-1])
    else:
        base = f"{FUEL}/{urllib.parse.quote(owner)}/models/{urllib.parse.quote(name)}"
        meta = json.loads(get(base))
        data = get(f"{base}/{meta['version']}/{urllib.parse.quote(name)}.zip")
        vdir = os.path.join(root, str(meta["version"]))
        zipfile.ZipFile(io.BytesIO(data)).extractall(vdir + ".part")
        os.replace(vdir + ".part", vdir)
        print(f"  model {name} v{meta['version']} ({len(data) / 1e6:.1f} MB, {meta.get('license_name', '?')})", flush=True)
    for dirpath, _, files in os.walk(vdir):  # models that reference meshes of other models
        for f in files:
            if f.endswith(".sdf"):
                for o, n in FUEL_REF.findall(open(os.path.join(dirpath, f), errors="ignore").read()):
                    fetch_model(o, n, done)


def main(worlds):
    for fname, task in CHECKPOINTS.items():
        path = os.path.join(ASSETS, "checkpoints", fname)
        if not os.path.exists(path):
            save(path, get(CHECKPOINT_URL.format(task)))
            print(f"  checkpoint {fname} ({task})", flush=True)
    done = set()
    for world in worlds:
        path = os.path.join(ASSETS, "subt", "worlds", world + ".sdf")
        if not os.path.exists(path):
            save(path, get(SUBT_URL.format(world)))
            print(f"  world {world}.sdf", flush=True)
        dot = path[:-4] + ".dot"  # tile connectivity graph (ground truth for exploration coverage), cave/tunnel worlds
        if not os.path.exists(dot):
            try:
                save(dot, get(SUBT_URL.format(world)[:-4] + ".dot", tries=2))
            except Exception:
                pass
        refs = sorted(set(FUEL_REF.findall(open(path).read())))
        print(f"  {world}: {len(refs)} fuel models", flush=True)
        for owner, name in refs:
            fetch_model(owner, name, done)
    print(f"assets ready in {ASSETS}")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sdf = ("<uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Cave Cap Type B</uri>"
               "<uri>https://fuel.gazebosim.org/1.0/openrobotics/models/rescue%20randy/tip/files/meshes/r.dae</uri>")
        assert FUEL_REF.findall(sdf) == [("OpenRobotics", "Cave Cap Type B"), ("openrobotics", "rescue%20randy")]
        print("selftest ok")
    else:
        main(sys.argv[1:] or ["simple_cave_01"])
