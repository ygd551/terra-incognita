"""Walking benchmark of a locomotion policy (default: the official ANYmal D rough-terrain policy) on graded terrain,
without ROS, localization or planners: how steep, how high and how rough the ground can be before it falls or gets
stuck: the baseline before fine-tuning (train_walk.py).

Every robot does one trial: stand 2 s, follow its command, stand until the end (22 s). Test cells, each repeated
--repeats times with the start pose jittered (+-0.3 m, +-10 deg):
  slope_up / slope_down / slope_cross  plane inclined 0-40 deg every 5: walk up, down, or along the contour (side slope)
  slope_turn                           same inclines: one arc revolution at vx 0.15, wz 0.5 (go2_sim's way of turning)
  stairs_up / stairs_down              pyramid stairs, steps 5-40 cm high, 0.3 m treads
  boxes                                grid of 0.45 m boxes, heights +-5-40 cm
  rough                                random uniform roughness 2-20 cm
Walk tests: 4 m at 0.5 and at 1.0 m/s. On the inclines the robot starts already standing on the slope (aligned to it).
Per trial: fall (base touches the ground, or body tilted > 60 deg from the ground normal) and when, progress (distance
made good along the start heading / commanded; turned angle / 360 deg for slope_turn), max tilt, share of time with a
thigh touching something, velocity tracking error. Summary (bench_report.py): walked = no fall before the end of
the command and progress >= 80 %; falls while standing still afterwards are counted apart.

  ./container.sh start      (and no ./sim.sh running: one Isaac Sim at a time)
  docker exec -it isaac_explore bash -c "cd /workspace/isaaclab && ./isaaclab.sh -p \\
      /workspace/isaac_explore/sim/bench_walk.py [--headless] [--name NAME] [--checkpoint PATH] [--repeats 16] \\
      [--tests slope_up ...] [--levels 25 30 ...]"
  -> logs/bench/NAME/trials.csv (one row per robot), summary.csv (one row per cell), summary.txt (the printed table)
Without --headless a window opens, the camera on the first robot's start (mouse: orbit, zoom); to watch, select a
few cells, e.g. --tests slope_up --levels 30 --repeats 4.
"""
import argparse

from isaaclab.app import AppLauncher

TESTS = ["slope_up", "slope_down", "slope_cross", "slope_turn", "stairs_up", "stairs_down", "boxes", "rough"]
parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--name", default="baseline", help="results go to logs/bench/NAME/")
parser.add_argument("--checkpoint", default=None, help="policy (default: ANYmal D's checkpoint in config/robots.yaml)")
parser.add_argument("--repeats", type=int, default=16, help="trials per test cell")
parser.add_argument("--tests", nargs="+", choices=TESTS, default=TESTS, help="only these tests (default: all)")
parser.add_argument("--levels", nargs="+", type=float, default=None,
                    help="only these levels: deg for slope_*, m for the others (default: all)")
parser.add_argument("--seed", type=int, default=0)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import csv  # noqa: E402
import math  # noqa: E402
import os  # noqa: E402
import time  # noqa: E402

import numpy as np  # noqa: E402
import torch  # noqa: E402
import yaml  # noqa: E402

import bench_report  # noqa: E402  (next to this script)
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

import isaaclab.terrains as terrain_gen  # noqa: E402
from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402
from isaaclab.terrains.height_field.hf_terrains_cfg import HfTerrainBaseCfg  # noqa: E402
from isaaclab.terrains.height_field.utils import height_field_to_mesh  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_from_euler_xyz, quat_mul  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from isaaclab_tasks.manager_based.locomotion.velocity.config.anymal_d.agents.rsl_rl_ppo_cfg import (  # noqa: E402
    AnymalDRoughPPORunnerCfg,
)
from isaaclab_tasks.manager_based.locomotion.velocity.config.anymal_d.rough_env_cfg import (  # noqa: E402
    AnymalDRoughEnvCfg_PLAY,
)

ROOT = "/workspace/isaac_explore"
ROBOT = yaml.safe_load(open(f"{ROOT}/config/robots.yaml"))["anymal_d"]
CHECKPOINT = os.path.join(ROOT, args.checkpoint or ROBOT["checkpoint"])  # relative: to the project
OUT = os.path.join(ROOT, "logs", "bench", args.name)

SLOPES = [0, 5, 10, 15, 20, 25, 30, 35, 40]                        # deg
STEPS = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40]           # m, stair height and box height
ROUGH = [0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15, 0.20]           # m, peak-to-peak roughness
SPEEDS = [0.5, 1.0]                                                # m/s; the planners use <= 1.0
DIST = 4.0                                                         # m walked per trial (tiles are 12 m)
TURN = (0.15, 0.5)                                                 # vx m/s, wz rad/s
SETTLE, T_END = 2.0, 22.0                                          # s
STAND_H = 0.6                                                      # base height above the ground at the start [m]
FALL_TILT = 60.0


@height_field_to_mesh
def incline_terrain(difficulty: float, cfg: "InclineCfg") -> np.ndarray:
    """A plane rising along +x at cfg.slope degrees, at z = 0 through the centre of the sub-terrain."""
    nx, ny = round(cfg.size[0] / cfg.horizontal_scale), round(cfg.size[1] / cfg.horizontal_scale)
    x = (np.arange(nx) - (nx - 1) / 2) * cfg.horizontal_scale
    z = math.tan(math.radians(cfg.slope)) * x / cfg.vertical_scale
    return np.rint(np.repeat(z[:, None], ny, axis=1)).astype(np.int16)


@configclass
class InclineCfg(HfTerrainBaseCfg):
    function = incline_terrain
    slope: float = 0.0  # deg


# ---------------- terrain: one 12 m tile per (kind, level), one column each ----------------
subs, tile = {}, {}  # sub-terrain configs in column order; (kind, level) -> column
for s in SLOPES:
    tile["incline", s], subs[f"incline_{s}"] = len(subs), InclineCfg(slope=s)
for h in STEPS:
    tile["stairs_up", h], subs[f"stairs_up_{h}"] = len(subs), terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
        step_height_range=(h, h), step_width=0.3, platform_width=3.0, border_width=1.0)
    tile["stairs_down", h], subs[f"stairs_down_{h}"] = len(subs), terrain_gen.MeshPyramidStairsTerrainCfg(
        step_height_range=(h, h), step_width=0.3, platform_width=3.0, border_width=1.0)
    tile["boxes", h], subs[f"boxes_{h}"] = len(subs), terrain_gen.MeshRandomGridTerrainCfg(
        grid_width=0.45, grid_height_range=(h, h), platform_width=2.0)
for h in ROUGH:
    tile["rough", h], subs[f"rough_{h}"] = len(subs), terrain_gen.HfRandomUniformTerrainCfg(
        noise_range=(0.0, h), noise_step=0.01, border_width=0.25)

# ---------------- test cells: (test, terrain kind, level, vx, wz, heading, walk time, expected distance) ------------
cells = []
for s in SLOPES:
    c = math.cos(math.radians(s))  # the command is along the body: up/down the slope, distance made good is horizontal
    for v in SPEEDS:
        cells += [("slope_up", "incline", s, v, 0.0, 0.0, DIST / v, DIST * c),
                  ("slope_down", "incline", s, v, 0.0, math.pi, DIST / v, DIST * c),
                  ("slope_cross", "incline", s, v, 0.0, math.pi / 2, DIST / v, DIST)]
    cells.append(("slope_turn", "incline", s, TURN[0], TURN[1], 0.0, 2 * math.pi / TURN[1], 2 * math.pi))
for h in STEPS:
    for v in SPEEDS:
        cells += [(test, test, h, v, 0.0, 0.0, DIST / v, DIST) for test in ("stairs_up", "stairs_down", "boxes")]
for h in ROUGH:
    cells += [("rough", "rough", h, v, 0.0, 0.0, DIST / v, DIST) for v in SPEEDS]
cells = [c for c in cells if c[0] in args.tests
         and (args.levels is None or any(abs(c[2] - x) < 1e-6 for x in args.levels))]
if not cells:
    raise SystemExit(f"[bench] no test cell matches --tests {args.tests} --levels {args.levels}")
N = len(cells) * args.repeats
cell_of = torch.arange(N) // args.repeats

# ---------------- environment: the official play config, our terrain, no resets, commands written every step -------
cfg = AnymalDRoughEnvCfg_PLAY()
cfg.seed = args.seed
cfg.scene.num_envs = N
cfg.scene.terrain.terrain_generator = terrain_gen.TerrainGeneratorCfg(
    size=(12.0, 12.0), border_width=5.0, num_rows=1, num_cols=len(subs), horizontal_scale=0.1, vertical_scale=0.005,
    slope_threshold=None,  # the default 0.75 (37 deg) would turn the steepest inclines into walls
    curriculum=True,       # with equal proportions: column k = sub-terrain k
    use_cache=False, seed=args.seed, sub_terrains=subs)
cfg.scene.terrain.max_init_terrain_level = None
cfg.curriculum.terrain_levels = None
# as in go2_sim (a sleeping robot ignores the policy). The falls when standing still are the policy's: the trials came
# out identical with the PhysX defaults of the official training (logs/bench/stand_*, 2026-10-23)
cfg.scene.robot.spawn.articulation_props.sleep_threshold = 0.0
cfg.scene.robot.spawn.articulation_props.stabilization_threshold = 0.0
cfg.events.reset_base.params["pose_range"] = {}  # the start pose is written below
cfg.events.reset_base.params["velocity_range"] = {}
cfg.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)  # default stance, not scaled 0.5-1.5
cfg.terminations.base_contact = None  # a fall is measured here, not reset
cfg.terminations.time_out = None
cfg.episode_length_s = 1e9
cmd_cfg = cfg.commands.base_velocity
cmd_cfg.resampling_time_range = (1e9, 1e9)
cmd_cfg.heading_command = False
cmd_cfg.rel_standing_envs = 0.0
cmd_cfg.rel_heading_envs = 0.0
env = ManagerBasedRLEnv(cfg)
venv = RslRlVecEnvWrapper(env)
runner = OnPolicyRunner(venv, AnymalDRoughPPORunnerCfg().to_dict(), log_dir=None, device=env.device)
runner.load(CHECKPOINT)
policy = runner.get_inference_policy(device=env.device)
dev = env.device
print(f"[bench] {len(cells)} cells x {args.repeats} = {N} robots, {len(subs)} terrain tiles, policy {CHECKPOINT}",
      flush=True)

# ---------------- per-robot parameters and start poses ----------------
def per_robot(i):
    return torch.tensor([cells[k][i] for k in cell_of.tolist()], device=dev, dtype=torch.float32)


level, vx, wz, heading0, t_walk, expected = (per_robot(i) for i in (2, 3, 4, 5, 6, 7))
col = torch.tensor([tile[cells[k][1], cells[k][2]] for k in cell_of.tolist()], device=dev)
incline = torch.tensor([cells[k][1] == "incline" for k in cell_of.tolist()], device=dev)
slope = torch.where(incline, torch.deg2rad(level), torch.zeros_like(level))
turn_test = torch.tensor([cells[k][0] == "slope_turn" for k in cell_of.tolist()], device=dev)

gen = torch.Generator(device=dev).manual_seed(args.seed)
jitter = (torch.rand(N, 2, device=dev, generator=gen) * 2 - 1) * 0.3
yaw = heading0 + (torch.rand(N, device=dev, generator=gen) * 2 - 1) * math.radians(10.0)
ground = env.scene.terrain.terrain_origins[0, col].clone()  # tile centre (stairs, boxes: the platform top)
ground[:, :2] += jitter
ground[:, 2] = torch.where(incline, torch.tan(slope) * jitter[:, 0], ground[:, 2])  # inclines pass z = 0 at the centre
normal = torch.stack([-torch.sin(slope), torch.zeros_like(slope), torch.cos(slope)], -1)  # ground normal
zero = torch.zeros_like(yaw)
quat = quat_mul(quat_from_euler_xyz(zero, -slope, zero), quat_from_euler_xyz(zero, zero, yaw))  # yaw, then the slope
robot = env.scene["robot"]
robot.write_root_pose_to_sim(torch.cat([ground + STAND_H * normal, quat], -1))
robot.write_root_velocity_to_sim(torch.zeros(N, 6, device=dev))
if env.viewport_camera_controller is not None:  # window open: look at the first robot's start from behind, above
    p, h = (ground[0] + STAND_H * normal[0]).tolist(), yaw[0].item()
    env.viewport_camera_controller.update_view_location(
        eye=[p[0] - 6 * math.cos(h), p[1] - 6 * math.sin(h), p[2] + 4], lookat=p)

# ---------------- run ----------------
contacts = env.scene["contact_forces"]
base_id = contacts.find_bodies("base")[0][0]
thigh_ids = contacts.find_bodies(".*THIGH")[0]
cmd_term = env.command_manager.get_term("base_velocity")
up_b = torch.tensor([0.0, 0.0, 1.0], device=dev).expand(N, 3)
direction = torch.stack([torch.cos(yaw), torch.sin(yaw)], -1)
fall_t = torch.full((N,), float("nan"), device=dev)
max_tilt, thigh_steps, active_steps, v_err, w_err, turned = (torch.zeros(N, device=dev) for _ in range(6))
start_xy, end_xy = torch.zeros(N, 2, device=dev), torch.zeros(N, 2, device=dev)
prev_heading = robot.data.heading_w.clone()
obs, _ = venv.get_observations()
t0 = time.time()
with torch.inference_mode():
    for k in range(round(T_END / env.step_dt)):
        t = k * env.step_dt
        alive = torch.isnan(fall_t)
        active = (t >= SETTLE) & (t < SETTLE + t_walk) & alive
        cmd = torch.zeros(N, 3, device=dev)
        cmd[:, 0], cmd[:, 2] = torch.where(active, vx, 0.0), torch.where(active, wz, 0.0)
        cmd_term.vel_command_b[:] = cmd
        obs = venv.step(policy(obs))[0]
        d = robot.data
        if k == round(SETTLE / env.step_dt) - 1:
            start_xy[:] = d.root_pos_w[:, :2]
        end_xy[active] = d.root_pos_w[active, :2]
        dh = torch.remainder(d.heading_w - prev_heading + math.pi, 2 * math.pi) - math.pi
        turned += torch.where(active, dh, 0.0)
        prev_heading = d.heading_w.clone()
        tilt = torch.rad2deg(torch.acos((quat_apply(d.root_quat_w, up_b) * normal).sum(-1).clamp(-1.0, 1.0)))
        max_tilt = torch.where(alive, torch.maximum(max_tilt, tilt), max_tilt)
        base_hit = contacts.data.net_forces_w_history[:, :, base_id].norm(dim=-1).amax(dim=1) > 1.0
        fall_t = torch.where(alive & (base_hit | (tilt > FALL_TILT)), torch.full_like(fall_t, t), fall_t)
        thigh_hit = (contacts.data.net_forces_w_history[:, :, thigh_ids].norm(dim=-1).amax(dim=1) > 1.0).any(-1)
        thigh_steps += (active & thigh_hit).float()
        active_steps += active.float()
        v_err += torch.where(active, (cmd[:, :2] - d.root_lin_vel_b[:, :2]).norm(dim=-1), 0.0)
        w_err += torch.where(active, (cmd[:, 2] - d.root_ang_vel_b[:, 2]).abs(), 0.0)
print(f"[bench] {T_END:.0f} s simulated in {time.time() - t0:.0f} s", flush=True)

# ---------------- results ----------------
progress = torch.where(turn_test, turned, ((end_xy - start_xy) * direction).sum(-1)) / expected
n_act = active_steps.clamp(min=1)
res = {k: x.cpu().tolist() for k, x in dict(fall_t=fall_t, t_walk=t_walk, progress=progress,
                                            max_tilt=max_tilt, thigh=thigh_steps / n_act, v_err=v_err / n_act,
                                            w_err=w_err / n_act).items()}
rows = []
for i in range(N):
    test, kind, lev, v = cells[cell_of[i]][:4]
    ft = res["fall_t"][i]
    phase = "" if math.isnan(ft) else "settle" if ft < SETTLE else "walk" if ft < SETTLE + res["t_walk"][i] else "stand"
    rows.append(dict(test=test, level=lev, speed=v, repeat=i % args.repeats,
                     fell=int(not math.isnan(ft)), fall_time="" if math.isnan(ft) else round(ft, 2), fall_phase=phase,
                     progress=round(res["progress"][i], 3), max_tilt_deg=round(res["max_tilt"][i], 1),
                     thigh_contact=round(res["thigh"][i], 3), vel_err=round(res["v_err"][i], 3),
                     yaw_rate_err=round(res["w_err"][i], 3)))
os.makedirs(OUT, exist_ok=True)
with open(os.path.join(OUT, "trials.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)

open(os.path.join(OUT, "info.txt"), "w").write(
    f"policy {CHECKPOINT}, {args.repeats} trials per cell, seed {args.seed}\n")
bench_report.report(OUT)
print(f"[bench] results in logs/bench/{args.name}/", flush=True)
os._exit(0)  # ponytail: skips Isaac Sim's shutdown, which hung for minutes after a run (2026-10-23)
