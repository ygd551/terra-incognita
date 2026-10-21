"""Run leggedrobotics/terrain-generator (wave function collapse) with a bounded number of ray workers.
  terrain_gen/.venv/bin/python terrain_gen/generate.py --cfg indoor --enable_sdf --mesh_dir terrain_gen/out/indoor
All arguments are passed to terrain-generator's examples/generate_with_wfc.py.
The generator calls ray.init() with one worker per CPU (32 here, ~0.4 GB each -> OOM); we start ray first with
--workers N and its own ray.init(ignore_reinit_error=True) then reuses that."""
import os, runpy, sys
import ray
import trimesh

if not hasattr(trimesh.Trimesh, "as_open3d"):  # removed in trimesh 4; the generator was written for 3.17
    def _as_open3d(self):
        import open3d as o3d
        return o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(self.vertices), o3d.utility.Vector3iVector(self.faces))
    trimesh.Trimesh.as_open3d = property(_as_open3d)

HERE = os.path.dirname(os.path.abspath(__file__))
workers = 4
if "--workers" in sys.argv:
    i = sys.argv.index("--workers"); workers = int(sys.argv[i + 1]); del sys.argv[i:i + 2]
ray.init(num_cpus=workers, include_dashboard=False, log_to_driver=False)
example_dir = os.path.join(HERE, "terrain-generator", "examples")
sys.path.insert(0, example_dir)  # the example imports its configs package relative to examples/
sys.argv[0] = os.path.join(example_dir, "generate_with_wfc.py")
os.chdir(os.path.join(HERE, "terrain-generator"))  # mesh cache paths are relative to the repo
runpy.run_path(sys.argv[0], run_name="__main__")
