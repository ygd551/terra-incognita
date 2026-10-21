"""Unitree Go2 (or ANYmal D, --robot anymal_d) in a converted SubT scene, driven by /cmd_vel through the pretrained
Isaac Lab rough-terrain policy. Per-robot lidar mount, spawn height and policy: config/robots.yaml.

Publishes (ROS 2, sim time):
  /clock                    rosgraph_msgs/Clock
  /imu/data                 sensor_msgs/Imu            every physics step (200 Hz), frame "imu"
  /lidar/points             sensor_msgs/PointCloud2    10 Hz, frame "lidar", fields x y z intensity t(uint32 ns, Ouster style)
                            Ouster OS1-32 emulated with Isaac Lab's GPU RayCaster (the RTX lidar plugin needs driver
                            >= 570; host has 550). The scan sweeps 1/20 rev per physics step, so motion distortion and
                            per-point times are real. No intensity (0), hits only the static scene collider.
  /ground_truth/odom        nav_msgs/Odometry          50 Hz, frame "world" -> "base_gt"
Subscribes:
  /sim/reset                std_msgs/String            "here" (default): stand the robot back up in place,
                                                        "spawn": teleport back to --spawn. Restart the LIO afterwards
                                                        (the pose jump is not something it can track).
  /cmd_vel                  geometry_msgs/Twist        teleop; clamped to the policy's training range
  /cmd_vel_stamped          geometry_msgs/TwistStamped CMU local planner output (remapped from its /cmd_vel)

Started by ../sim.sh (see ../README.md), which runs inside the isaac_explore container:
  source /opt/ros/humble/setup.bash
  /workspace/isaaclab/isaaclab.sh -p /workspace/isaac_explore/sim/go2_sim.py [--headless] [--scene ...] [--robot anymal_d]
"""
import argparse
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--scene", default="/workspace/isaac_explore/sim/scenes/simple_cave_01.usd")
parser.add_argument("--robot", choices=["go2", "anymal_d"], default="go2")
parser.add_argument("--checkpoint", default=None, help="policy (default: checkpoint in config/robots.yaml)")
parser.add_argument("--spawn", type=float, nargs=3, default=None,
                    help="x y z in the scene [m] (default: 2 0 spawn_z from config/robots.yaml)")
parser.add_argument("--no_realtime", action="store_true", help="run as fast as possible instead of wall-clock pace")
parser.add_argument("--see_through", type=float, default=0.0, metavar="OPACITY",
                    help="draw the scene semi-transparent (0.25 works well) and let the GUI camera follow the robot, to watch it "
                         "inside the cave; "
                         "visual only - physics, lidar and height scan use the collision mesh. 0 = off")
parser.add_argument("--auto_reset", type=float, default=0.0, metavar="S",
                    help="stand the robot back up in place after S s (sim) lying down; 0 = never. For unattended runs "
                         "with explore.launch.py lio:=gt only: the pose jump breaks BIEVR")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import math  # noqa: E402
import os  # noqa: E402

import numpy as np  # noqa: E402
import yaml  # noqa: E402
import rclpy  # noqa: E402
import torch  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from geometry_msgs.msg import PointStamped, Twist, TwistStamped  # noqa: E402
from nav_msgs.msg import Odometry, Path  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402
from sensor_msgs.msg import Imu, PointCloud2, PointField  # noqa: E402
from std_msgs.msg import String  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402
from isaaclab.sensors import ImuCfg, RayCasterCfg, patterns  # noqa: E402
from isaaclab.terrains import TerrainImporterCfg  # noqa: E402
from isaaclab.utils.math import quat_apply_inverse, yaw_quat  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
if args.robot == "go2":
    from isaaclab_tasks.manager_based.locomotion.velocity.config.go2.agents.rsl_rl_ppo_cfg import (  # noqa: E402
        UnitreeGo2RoughPPORunnerCfg as RunnerCfg,
    )
    from isaaclab_tasks.manager_based.locomotion.velocity.config.go2.rough_env_cfg import (  # noqa: E402
        UnitreeGo2RoughEnvCfg_PLAY as EnvCfg,
    )
else:
    from isaaclab_tasks.manager_based.locomotion.velocity.config.anymal_d.agents.rsl_rl_ppo_cfg import (  # noqa: E402
        AnymalDRoughPPORunnerCfg as RunnerCfg,
    )
    from isaaclab_tasks.manager_based.locomotion.velocity.config.anymal_d.rough_env_cfg import (  # noqa: E402
        AnymalDRoughEnvCfg_PLAY as EnvCfg,
    )

ROBOT = yaml.safe_load(open("/workspace/isaac_explore/config/robots.yaml"))[args.robot]
# Lidar mount on the base link (= the imu frame), no rotation. BIEVR reads it from config/bievr_sim_<robot>.yaml.
LIDAR_POS = tuple(ROBOT["lidar_pos"])
CHECKPOINT = args.checkpoint or os.path.join("/workspace/isaac_explore", ROBOT["checkpoint"])
SPAWN = tuple(args.spawn) if args.spawn else (2.0, 0.0, ROBOT["spawn_z"])
# |vx|, |vy|, |wz|. Trained range is +-1, but pure in-place turns at 1 rad/s make the pretrained policy stumble
# (IMU spikes of 160 m/s^2, falls); 0.6 rad/s is calm.
CMD_LIMITS = np.array([1.0, 1.0, 0.6])
# Slew limits on the command fed to the policy [m/s^2, m/s^2, rad/s^2]. The policy was trained with commands that
# change every 10 s; the planner can flip wz between +1 and -1 from one message to the next, which tipped it over.
CMD_SLEW = np.array([1.0, 1.0, 2.0])
# Stuck recovery: asked to go forward (> STUCK_CMD) for STUCK_TIME s while really moving < STUCK_VEL -> back up, then turn.
# ponytail: uses the sim's true body velocity and a fixed manoeuvre; on a real robot use odometry and let the
# planner pick the escape direction.
# STUCK_CMD must stay below SPIN_CREEP: an arc turn that creeps into a rock (not collision-checked) must count as
# stuck, otherwise the robot scrabbles on the rock until BIEVR diverges (seen at the cave tunnel mouth).
STUCK_CMD, STUCK_VEL, STUCK_TIME = 0.1, 0.05, 4.0
RECOVERY = [(1.5, (-0.3, 0.0, 0.0)), (1.5, (0.0, 0.0, 0.6))]  # (seconds, command) steps
# Planner stall: the CMU local planner found no free path (it then publishes a 1-pose /path and zero speed) for
# PLANNER_STALL_TIME s while the way point is still > 1.5 m away -> same RECOVERY manoeuvre. The stuck check above
# cannot see this case (no forward command). ponytail: fixed back-up-and-turn-left; the planner could pick the side.
PLANNER_STALL_TIME, PLANNER_STALL_DIST = 5.0, 1.5
# Turn in an arc instead of on the spot: BIEVR-LIO sometimes diverges while the robot shuffles in place (little
# translation, lots of gait sway), but tracks walking turns reliably. ponytail: fixed creep speed; a planner that
# knows the obstacle clearance should choose it instead.
SPIN_CREEP = 0.15  # m/s added to pure turn commands (|vx| < SPIN_CREEP and |wz| > 0.1)
# Slow down in tight turns: the CMU path follower keeps full speed while it saturates the yaw rate (0.5 rad/s), so at
# 1 m/s its turning radius is 2 m and a way point 1.5-2 m to the side is circled forever (seen right after raising
# the top speed). vx is capped from full at |wz| <= TURN_WZ[0] down to TURN_VX at |wz| >= TURN_WZ[1] (radius ~1 m,
# the behaviour validated at 0.5 m/s). ponytail: speed-from-yaw-rate rule; a curvature-aware follower is the fix.
TURN_VX, TURN_WZ = 0.5, (0.2, 0.4)
SCENE_MESH = "/World/ground/terrain/collision"  # sdf_to_usd.py bakes all colliders into this one mesh
LIDAR_HZ, LIDAR_CH, LIDAR_COLS, LIDAR_VFOV = 10, 32, 1024, (-22.5, 22.5)  # Ouster OS1-32, 10 Hz, 1024 columns

# ---------------- environment ----------------
cfg = EnvCfg()
cfg.scene.num_envs = 1
cfg.scene.terrain = TerrainImporterCfg(prim_path="/World/ground", terrain_type="usd", usd_path=args.scene)
cfg.scene.height_scanner.mesh_prim_paths = [SCENE_MESH]  # RayCaster supports exactly one mesh
# Training casts the height scan from 20 m above the base; under a cave roof those rays hit the rock's top surface
# and the policy sees a 16 m wall (it froze and tipped over at the cave mouth). The observation is
# base_z - hit_z - 0.5, independent of the ray start height, so starting just above the base is equivalent
# whenever there is a ceiling - and blind to anything taller than 0.5 m above the base (fine: clipped at 1 m anyway).
cfg.scene.height_scanner.offset.pos = (0.0, 0.0, 0.5)
cfg.scene.imu = ImuCfg(prim_path="{ENV_REGEX_NS}/Robot/base")
cfg.scene.lidar = RayCasterCfg(
    prim_path="{ENV_REGEX_NS}/Robot/base", mesh_prim_paths=[SCENE_MESH], max_distance=100.0,
    offset=RayCasterCfg.OffsetCfg(pos=LIDAR_POS), ray_alignment="base",
    pattern_cfg=patterns.LidarPatternCfg(channels=LIDAR_CH, vertical_fov_range=LIDAR_VFOV,
                                         horizontal_fov_range=(-180.0, 180.0), horizontal_res=360.0 / LIDAR_COLS))
cfg.scene.robot.init_state.pos = SPAWN
# Never let PhysX put the robot to sleep: after standing still for a while it fell asleep, and the policy's joint
# targets (written through the tensor API) did not wake it - the Go2 froze in place and ignored every command.
cfg.scene.robot.spawn.articulation_props.sleep_threshold = 0.0
cfg.scene.robot.spawn.articulation_props.stabilization_threshold = 0.0
cfg.curriculum.terrain_levels = None  # needs the procedural terrain generator
cfg.events.push_robot = None
cfg.events.reset_base.params["pose_range"] = {}
cfg.events.reset_base.params["velocity_range"] = {}
cfg.terminations.base_contact = None  # never auto-reset: a fall is something to observe, not hide
cfg.terminations.time_out = None
cfg.episode_length_s = 1e9
cmd_cfg = cfg.commands.base_velocity
cmd_cfg.resampling_time_range = (1e9, 1e9)  # command is written from /cmd_vel every step
cmd_cfg.heading_command = False
cmd_cfg.rel_standing_envs = 0.0
cmd_cfg.rel_heading_envs = 0.0

if args.see_through > 0:  # the GUI camera follows the robot, looking down through the cave ceiling
    cfg.viewer.origin_type, cfg.viewer.asset_name = "asset_root", "robot"
    cfg.viewer.eye, cfg.viewer.lookat = (-1.5, -1.5, 10.0), (0.0, 0.0, 0.0)
env = ManagerBasedRLEnv(cfg)
if args.see_through > 0:
    # one translucent material over the whole scene subtree, overriding the meshes' own materials (the robot lives
    # under /World/envs and keeps its look). Isaac Lab's render presets switch RTX translucency off; without it any
    # opacity < 1 renders the rock fully invisible. Dark blue stays visible against the bright sky.
    import carb  # noqa: E402

    carb.settings.get_settings().set("/rtx/translucency/enabled", True)
    carb.settings.get_settings().set("/rtx/raytracing/fractionalCutoutOpacity", True)
    see_through = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.15, 0.3, 0.6), roughness=1.0, opacity=args.see_through)
    see_through.func("/World/Looks/see_through", see_through)
    sim_utils.bind_visual_material("/World/ground", "/World/Looks/see_through", stronger_than_descendants=True)
venv = RslRlVecEnvWrapper(env)
runner = OnPolicyRunner(venv, RunnerCfg().to_dict(), log_dir=None, device=env.device)
runner.load(CHECKPOINT)
policy = runner.get_inference_policy(device=env.device)

# ---------------- ROS 2 ----------------
rclpy.init()
node = rclpy.create_node("go2_sim")
pub_clock = node.create_publisher(Clock, "/clock", 10)
pub_imu = node.create_publisher(Imu, "/imu/data", 200)
pub_pc = node.create_publisher(PointCloud2, "/lidar/points", 5)
pub_gt = node.create_publisher(Odometry, "/ground_truth/odom", 10)
cmd = np.zeros(3)  # latest requested command
cmd_applied = np.zeros(3)  # slew-limited command actually given to the policy


def on_cmd(msg):
    cmd[:] = np.clip([msg.linear.x, msg.linear.y, msg.angular.z], -CMD_LIMITS, CMD_LIMITS)


node.create_subscription(Twist, "/cmd_vel", on_cmd, 10)
node.create_subscription(TwistStamped, "/cmd_vel_stamped", lambda m: on_cmd(m.twist), 10)
reset_req = []
node.create_subscription(String, "/sim/reset", lambda m: reset_req.append(m.data.strip() or "here"), 1)
plan = {}  # latest local path length, way point and robot position, all in the planner's map frame
node.create_subscription(Path, "/path", lambda m: plan.__setitem__("n", len(m.poses)), 5)
node.create_subscription(PointStamped, "/way_point", lambda m: plan.__setitem__("wp", (m.point.x, m.point.y)), 5)
node.create_subscription(Odometry, "/state_estimation",
                         lambda m: plan.__setitem__("pos", (m.pose.pose.position.x, m.pose.pose.position.y)), 5)


def reset_robot(mode, log=True):
    """Upright the robot (keeping position + heading) or send it back to the spawn pose, joints at default."""
    robot = env.scene["robot"]
    d = robot.data
    if mode == "spawn":
        pose = d.default_root_state[:, :7].clone()
        pose[:, :3] += env.scene.env_origins
    else:
        pose = torch.cat([d.root_pos_w + torch.tensor([0.0, 0.0, 0.3], device=env.device), yaw_quat(d.root_quat_w)], -1)
    robot.write_root_pose_to_sim(pose)
    robot.write_root_velocity_to_sim(torch.zeros_like(d.root_vel_w))
    robot.write_joint_state_to_sim(d.default_joint_pos.clone(), d.default_joint_vel.clone())
    robot.reset()
    env.action_manager.reset()
    if log:
        print(f"[go2_sim] robot reset ({mode}) to {pose[0, :3].tolist()}. Restart the LIO now.", flush=True)


def stamp(t):
    ns = int(round(t * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def publish_imu(t):
    d = env.scene["imu"].data
    m = Imu()
    m.header.stamp, m.header.frame_id = stamp(t), "imu"
    w, a, q = d.ang_vel_b[0].tolist(), d.lin_acc_b[0].tolist(), d.quat_w[0].tolist()  # quat wxyz
    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = w
    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = a
    m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = q
    pub_imu.publish(m)
    c = Clock()
    c.clock = m.header.stamp
    pub_clock.publish(c)


def publish_gt(t):
    r = env.scene["robot"].data
    m = Odometry()
    m.header.stamp, m.header.frame_id, m.child_frame_id = stamp(t), "world", "base_gt"
    p, q = r.root_pos_w[0].tolist(), r.root_quat_w[0].tolist()
    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = p
    m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z = q
    v, w = r.root_lin_vel_b[0].tolist(), r.root_ang_vel_b[0].tolist()
    m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = v
    m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z = w
    pub_gt.publish(m)


PC_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("t", "<u4")])
PC_FIELDS = [PointField(name=n, offset=PC_DTYPE.fields[n][1], datatype=dt, count=1)
             for n, dt in (("x", 7), ("y", 7), ("z", 7), ("intensity", 7), ("t", 6))]
# rolling scan: each physics step casts only the azimuth slice the spinning head is at, in the sensor frame of
# that instant (exactly how a real spinning lidar distorts under motion)
lidar = env.scene["lidar"]
n_slices = round(1.0 / (LIDAR_HZ * env.physics_dt))  # 20 at 200 Hz physics
az = torch.atan2(lidar.ray_directions[0, :, 1], lidar.ray_directions[0, :, 0])  # [-pi, pi)
slice_rays = [torch.nonzero(((az + math.pi) / (2 * math.pi) * n_slices).long().clamp(max=n_slices - 1) == k)[:, 0]
              for k in range(n_slices)]
scan = {"k": 0, "t0": 0.0, "pts": [], "ts": []}
lidar_offset = torch.tensor(LIDAR_POS, device=env.device)  # mount rotation is identity


def lidar_step(t):
    """Call after every physics step: cast the current slice, publish when the revolution completes."""
    k = scan["k"]
    if k == 0:
        scan["t0"], scan["pts"], scan["ts"] = t, [], []
    d = lidar.data
    hits = d.ray_hits_w[0, slice_rays[k]]
    hits = hits[torch.isfinite(hits).all(-1)]
    # RayCaster's data.pos_w/quat_w are the *base* pose (the mount offset is only baked into the ray starts), so
    # subtract LIDAR_POS to get points in the lidar frame. Without this the cloud was 0.18 m off the extrinsic in
    # config/bievr_sim_go2.yaml: invisible when walking straight, but every in-place turn made BIEVR-LIO diverge.
    p = quat_apply_inverse(d.quat_w[0].expand(len(hits), 4), hits - d.pos_w[0]) - lidar_offset
    scan["pts"].append(p)
    scan["ts"].append(torch.full((len(p),), t - scan["t0"], device=p.device))
    scan["k"] = (k + 1) % n_slices
    if scan["k"] == 0:
        publish_scan(torch.cat(scan["pts"]).cpu().numpy(), torch.cat(scan["ts"]).cpu().numpy(), scan["t0"])


def publish_scan(xyz, dt, t0):
    pc = np.zeros(len(xyz), PC_DTYPE)
    pc["x"], pc["y"], pc["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    pc["t"] = np.round(dt * 1e9).astype(np.uint32)
    m = PointCloud2()
    m.header.stamp, m.header.frame_id = stamp(t0), "lidar"  # Ouster convention: stamp = scan start
    m.height, m.width = 1, len(pc)
    m.fields, m.is_bigendian, m.point_step = PC_FIELDS, False, PC_DTYPE.itemsize
    m.row_step, m.is_dense, m.data = PC_DTYPE.itemsize * len(pc), True, pc.tobytes()
    pub_pc.publish(m)


# ---------------- main loop: ManagerBasedRLEnv.step() minus rewards/resets, IMU at physics rate ----------------
cmd_term = env.command_manager.get_term("base_velocity")
obs, _ = venv.get_observations()
# Settle before publishing: the spawn drop + landing would poison LIO's static IMU init window. At zero command the
# pretrained policy sometimes topples in its first seconds (ANYmal: ~1 in 3 starts), so "[go2_sim] running" waits until
# the robot has stood upright and still for SETTLE_TIME, putting it back on the spawn pose after a fall. No LIO is
# running yet at this point, so the reset costs nothing.
SETTLE_TIME, SETTLE_MAX = 3.0, 10.0  # s (sim) of quiet standing required; give up on an attempt after SETTLE_MAX
cmd_term.vel_command_b[:] = 0.0
with torch.inference_mode():
    for attempt in range(1, 6):
        quiet = 0.0
        for _ in range(int(SETTLE_MAX / env.step_dt)):
            obs = venv.step(policy(obs))[0]
            d = env.scene["robot"].data
            up_z = d.projected_gravity_b[0, 2].item()  # -1 upright
            if up_z > -0.5:
                break  # fallen
            still = up_z < -0.95 and d.root_lin_vel_b[0, :2].norm().item() < 0.1
            quiet = quiet + env.step_dt if still else 0.0
            if quiet >= SETTLE_TIME:
                break
        if quiet >= SETTLE_TIME:
            break
        print(f"[go2_sim] did not stand still after spawning (attempt {attempt}): back to the spawn pose", flush=True)
        reset_robot("spawn", log=False)
        obs, _ = venv.get_observations()
wall0, sim0 = time.time(), env.sim.current_time
print(f"[go2_sim] running: robot={args.robot} scene={args.scene}. Drive with /cmd_vel.", flush=True)
fallen = False
falls, fallen_at = 0, float("inf")  # fall counter (printed), sim time of the current fall
stuck_t = 0.0  # how long we have been commanded forward without moving
stall_t = 0.0  # how long the local planner has had no free path
recovery = []  # remaining (seconds, command) steps of an active recovery manoeuvre
while app.is_running():
    rclpy.spin_once(node, timeout_sec=0.0)
    if reset_req:
        with torch.inference_mode():  # sim state tensors are inference tensors
            reset_robot(reset_req.pop())
        cmd[:] = 0.0
        cmd_applied[:] = 0.0
    up_z = env.scene["robot"].data.projected_gravity_b[0, 2].item()  # -1 upright, +1 upside down
    if (up_z > -0.5) != fallen:
        fallen = not fallen
        if fallen:
            cmd[:] = 0.0  # stop driving a robot that is lying on its back
            falls += 1
            fallen_at = env.sim.current_time
            print(f"[go2_sim] robot has fallen (fall #{falls} at t={fallen_at:.1f} s). Stand it up with:  ros2 topic pub "
                  "--once /sim/reset std_msgs/msg/String \"{data: here}\"   (or data: spawn)", flush=True)
    if fallen:
        cmd[:] = 0.0
        # opt-in for unattended runs (ground-truth localization: a reset pose jump would break BIEVR)
        if args.auto_reset > 0 and env.sim.current_time - fallen_at >= args.auto_reset and not reset_req:
            print(f"[go2_sim] auto reset after {args.auto_reset:.0f} s down (fall #{falls})", flush=True)
            reset_req.append("here")
            fallen_at = float("inf")  # once per fall; the reset clears `fallen` on the next upright check
    if not recovery and abs(cmd[0]) < SPIN_CREEP and abs(cmd[2]) > 0.1:
        cmd[0] = SPIN_CREEP
    if not recovery and cmd[0] > TURN_VX:
        f = min(max((abs(cmd[2]) - TURN_WZ[0]) / (TURN_WZ[1] - TURN_WZ[0]), 0.0), 1.0)  # 0 straight .. 1 tight turn
        cmd[0] = min(cmd[0], CMD_LIMITS[0] - f * (CMD_LIMITS[0] - TURN_VX))
    v_body = env.scene["robot"].data.root_lin_vel_b[0, 0].item()
    stuck_t = stuck_t + env.step_dt if (cmd[0] > STUCK_CMD and abs(v_body) < STUCK_VEL and not recovery) else 0.0
    if stuck_t > STUCK_TIME and not fallen:
        print(f"[go2_sim] stuck (commanded {cmd[0]:.2f} m/s, moving {v_body:.2f} m/s): backing up and turning", flush=True)
        recovery, stuck_t = [list(step) for step in RECOVERY], 0.0
    stalled = (plan.get("n", 2) <= 1 and "wp" in plan and "pos" in plan and not recovery and not fallen
               and math.dist(plan["wp"], plan["pos"]) > PLANNER_STALL_DIST)
    stall_t = stall_t + env.step_dt if stalled else 0.0
    if stall_t > PLANNER_STALL_TIME:
        print(f"[go2_sim] local planner found no path for {PLANNER_STALL_TIME:.0f} s (way point "
              f"{math.dist(plan['wp'], plan['pos']):.1f} m away): backing up and turning", flush=True)
        recovery, stall_t = [list(step) for step in RECOVERY], 0.0
    if recovery:  # overrides the planner until the manoeuvre is done
        cmd[:] = recovery[0][1]
        recovery[0][0] -= env.step_dt
        if recovery[0][0] <= 0:
            recovery.pop(0)
    cmd_applied += np.clip(cmd - cmd_applied, -CMD_SLEW * env.step_dt, CMD_SLEW * env.step_dt)
    cmd_term.vel_command_b[:] = torch.tensor(cmd_applied, device=env.device, dtype=torch.float32)
    with torch.inference_mode():
        env.action_manager.process_action(policy(obs))
        for _ in range(env.cfg.decimation):
            env._sim_step_counter += 1
            env.action_manager.apply_action()
            env.scene.write_data_to_sim()
            env.sim.step(render=False)
            if env._sim_step_counter % env.cfg.sim.render_interval == 0:
                env.sim.render()
            env.scene.update(dt=env.physics_dt)
            publish_imu(env.sim.current_time)
            lidar_step(env.sim.current_time)
        t = env.sim.current_time
        publish_gt(t)
        obs = env.observation_manager.compute(update_history=True)["policy"]
    if not args.no_realtime:
        lag = (t - sim0) - (time.time() - wall0)
        if lag > 0:
            time.sleep(lag)

node.destroy_node()
rclpy.shutdown()
env.close()
app.close()
