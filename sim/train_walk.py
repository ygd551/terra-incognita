"""Fine-tune the ANYmal D walking policy (Isaac Lab's rough-terrain PPO with rsl_rl) from the official weights on
harder ground (measured with bench_walk.py before and after). Network, observations (235) and actions (12) stay the
official ones, so every checkpoint drops into go2_sim.py (./sim.sh --checkpoint PATH) and bench_walk.py
(--checkpoint PATH) as it is.

Terrain (--terrain):
  official  Isaac Lab's rough mix: stairs and boxes 5-23 cm, roughness 2-10 cm, slopes up to 22 deg
  steep     half the official mix + half harder ground: slopes 19-39 deg (up and down), slopes 11-35 deg covered
            with 2-10 cm roughness, boxes 10-30 cm, stairs 10-30 cm (up and down). The 35 deg target is close to the
            physical limit of the foot friction (static 0.8 = tan 39 deg, dynamic 0.6 = tan 31 deg)
  10 difficulty rows with the official curriculum: a robot that walked far enough goes one row harder, else easier.
Commands: the official training's, except --standing (share of robots told to stand still; official 0.02) and
--heading_envs (share whose turn rate follows a target heading; official 1.0, the others get a turn rate directly, as
from the planners). Rewards, friction (static 0.8 / dynamic 0.6) and the rest: unchanged.

  ./container.sh start      (and no ./sim.sh or bench_walk.py running: one Isaac Sim at a time)
  docker exec -it isaac_explore bash -c "cd /workspace/isaaclab && ./isaaclab.sh -p \\
      /workspace/isaac_explore/sim/train_walk.py --headless --name NAME [--iterations 1000] [--terrain steep] \\
      [--standing 0.02] [--heading_envs 1.0] [--checkpoint PATH] [--num_envs 4096]"
  -> logs/train/NAME/: model_<iteration>.pt every 50 iterations and at the end (the official weights are iteration
     1499, so 1000 more end at model_2498.pt), TensorBoard logs, params/ (the full configuration), info.txt
Without --headless a window opens; for a look at the terrain use few robots and iterations:
--num_envs 64 --iterations 5.
"""
import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--name", required=True, help="run name: results go to logs/train/NAME/")
parser.add_argument("--iterations", type=int, default=1000, help="PPO iterations to train (24 steps x num_envs each)")
parser.add_argument("--terrain", choices=["official", "steep"], default="steep")
parser.add_argument("--standing", type=float, default=0.02, help="share of robots commanded to stand still")
parser.add_argument("--heading_envs", type=float, default=1.0,
                    help="share of robots whose turn rate follows a target heading (the rest: a turn rate command)")
parser.add_argument("--checkpoint", default=None,
                    help="start from this checkpoint (default: the official ANYmal D one in config/robots.yaml)")
parser.add_argument("--num_envs", type=int, default=4096)
parser.add_argument("--seed", type=int, default=42)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
LOG_DIR = os.path.join("/workspace/isaac_explore", "logs", "train", args.name)
if os.path.exists(LOG_DIR):  # checked before Isaac Sim starts
    raise SystemExit(f"logs/train/{args.name} exists: pick another --name (to continue a run: --checkpoint "
                     f"logs/train/{args.name}/model_<iteration>.pt with a new --name)")
app = AppLauncher(args).app

import numpy as np  # noqa: E402
import yaml  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

import isaaclab.terrains as terrain_gen  # noqa: E402
from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402
from isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG  # noqa: E402
from isaaclab.terrains.height_field import hf_terrains  # noqa: E402
from isaaclab.terrains.height_field.utils import height_field_to_mesh  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.io import dump_yaml  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from isaaclab_tasks.manager_based.locomotion.velocity.config.anymal_d.agents.rsl_rl_ppo_cfg import (  # noqa: E402
    AnymalDRoughPPORunnerCfg,
)
from isaaclab_tasks.manager_based.locomotion.velocity.config.anymal_d.rough_env_cfg import (  # noqa: E402
    AnymalDRoughEnvCfg,
)

ROOT = "/workspace/isaac_explore"
ROBOT = yaml.safe_load(open(f"{ROOT}/config/robots.yaml"))["anymal_d"]
CHECKPOINT = os.path.join(ROOT, args.checkpoint or ROBOT["checkpoint"])  # relative: to the project


@height_field_to_mesh
def rough_pyramid_slope(difficulty: float, cfg: "RoughSlopeCfg") -> np.ndarray:
    """Isaac Lab's pyramid slope with uniform random roughness on top (rubble on a slope)."""
    hf = hf_terrains.pyramid_sloped_terrain.__wrapped__(difficulty, cfg)
    amp = cfg.noise_range[0] + difficulty * (cfg.noise_range[1] - cfg.noise_range[0])
    return hf + np.rint(np.random.uniform(0.0, amp, hf.shape) / cfg.vertical_scale).astype(np.int16)


@configclass
class RoughSlopeCfg(terrain_gen.HfPyramidSlopedTerrainCfg):
    function = rough_pyramid_slope
    noise_range: tuple[float, float] = (0.02, 0.10)  # m peak to peak, from the easiest to the hardest row


cfg = AnymalDRoughEnvCfg()
cfg.seed = args.seed
cfg.scene.num_envs = args.num_envs
if args.terrain == "steep":
    # proportions in steps of 0.05 = one of the 20 columns each
    official = {k: v.replace(proportion=v.proportion / 2) for k, v in ROUGH_TERRAINS_CFG.sub_terrains.items()}
    steep = (0.35, 0.8)  # slope = tan: 19-39 deg
    harder = {
        "steep_slope": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.1, slope_range=steep, platform_width=2.0, border_width=0.25),
        "steep_slope_inv": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.1, slope_range=steep, platform_width=2.0, border_width=0.25),
        "rough_slope": RoughSlopeCfg(proportion=0.05, slope_range=(0.2, 0.7), platform_width=2.0, border_width=0.25),
        "rough_slope_inv": RoughSlopeCfg(
            proportion=0.05, slope_range=(0.2, 0.7), platform_width=2.0, border_width=0.25, inverted=True),
        "high_boxes": terrain_gen.MeshRandomGridTerrainCfg(
            proportion=0.1, grid_width=0.45, grid_height_range=(0.1, 0.3), platform_width=2.0),
        "high_stairs": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.05, step_height_range=(0.1, 0.3), step_width=0.3, platform_width=3.0, border_width=1.0),
        "high_stairs_inv": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.05, step_height_range=(0.1, 0.3), step_width=0.3, platform_width=3.0, border_width=1.0),
    }
    cfg.scene.terrain.terrain_generator = ROUGH_TERRAINS_CFG.replace(
        sub_terrains={**official, **harder}, curriculum=True,
        slope_threshold=None)  # the default 0.75 turns slopes over 37 deg into walls; bench_walk.py has none either
cmd = cfg.commands.base_velocity
cmd.rel_standing_envs = args.standing
cmd.rel_heading_envs = args.heading_envs

agent = AnymalDRoughPPORunnerCfg()
agent.seed = args.seed
agent.max_iterations = args.iterations
agent.experiment_name, agent.run_name = "anymal_d_walk", args.name
env = RslRlVecEnvWrapper(ManagerBasedRLEnv(cfg), clip_actions=agent.clip_actions)
runner = OnPolicyRunner(env, agent.to_dict(), log_dir=LOG_DIR, device=agent.device)
runner.load(CHECKPOINT)  # weights and optimizer state; continues the iteration count
dump_yaml(os.path.join(LOG_DIR, "params", "env.yaml"), cfg)
dump_yaml(os.path.join(LOG_DIR, "params", "agent.yaml"), agent)
open(os.path.join(LOG_DIR, "info.txt"), "w").write(
    f"from {CHECKPOINT} (iteration {runner.current_learning_iteration}), {args.iterations} iterations, "
    f"{args.num_envs} robots, terrain {args.terrain}, standing {args.standing}, heading_envs {args.heading_envs}, "
    f"seed {args.seed}\n")
print(f"[train] {open(os.path.join(LOG_DIR, 'info.txt')).read().strip()} -> logs/train/{args.name}/", flush=True)
runner.learn(num_learning_iterations=args.iterations, init_at_random_ep_len=True)
runner.writer.flush()  # TensorBoard writes every 10 s; the exit below skips the rest
print(f"[train] done: logs/train/{args.name}/model_{runner.current_learning_iteration}.pt", flush=True)
os._exit(0)  # ponytail: skips Isaac Sim's shutdown, which can hang for minutes (as in bench_walk.py)
