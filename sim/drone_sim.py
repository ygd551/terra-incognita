"""RMF-Obelix multirotor (NTNU's Resilient Micro Flyer, as modelled in ntnu-arl/rotors_simulator) in a converted
scene. One rigid body: the thrust of each rotor where it sits (roll and pitch torques), the rotor drag torque (yaw),
first-order motor lag and rotor drag against the body's sideways velocity, all with RotorS's parameters
(config/robots.yaml). RotorS's Lee geometric position controller (same equations and gains) flies the velocity commands
like a flight controller in position mode (e.g. PX4's): while commanded or still moving its position reference is the
drone itself (velocity control alone: a reference running ahead made it overshoot by 40 % to catch up);
once stopped it holds the position where it stopped. The sensors and their topics are the legged
robots' (sim_common.py), so BIEVR-LIO, the ground-truth odometry and the planners' adapters work unchanged.

Publishes (ROS 2, sim time): /clock, /imu/data (200 Hz), /lidar/points (10 Hz; the robot's lidar in config/robots.yaml,
  rmf_obelix_ouster64: Ouster OS0-64, rmf_obelix_avia: Livox Avia; --lidar overrides it),
  /ground_truth/odom (50 Hz), /camera/image_raw + /camera/camera_info (the front camera, 10 Hz; --no_camera: off)
Light: --light dark (default) lights the scene only with the drone's LED, next to the camera, as underground; --light
  bright puts a dome light over everything, to watch the flight in the GUI.
Subscribes:
  /cmd_vel (Twist) or /cmd_vel_stamped (TwistStamped): velocity in the heading frame (x forward, y left, z up) and yaw
      rate. The drone waits on the floor with the motors off until the first nonzero command; without a command for
      CMD_TIMEOUT s it hovers where it is.
  /sim/reset (String): "spawn" = back to the start, landed, motors off; "here" (default) = upright where it is, hovering.
Prints when the motors start, every collision while flying, and a crash (tilted > 60 deg: motors off).

Started by ../sim.sh --robot rmf_obelix_ouster64|rmf_obelix_avia (runs inside the isaac_explore container); the asset comes from
tools/add_drone.sh. Fly by hand: python3 /isaac_explore/sim/drone_test.py (see there).
"""
import argparse
import os
import time

import yaml
from isaaclab.app import AppLauncher

LIDARS = yaml.safe_load(open("/workspace/isaac_explore/config/lidars.yaml"))
AERIAL = [k for k, v in yaml.safe_load(open("/workspace/isaac_explore/config/robots.yaml")).items()
          if v.get("type") == "aerial"]
parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--robot", choices=AERIAL, default="rmf_obelix_ouster64")
parser.add_argument("--scene", default="simple_cave_01",
                    help="scene name (sim/scenes/NAME.usd; start pose defaults in config/scenes.yaml) or a .usd path")
parser.add_argument("--lidar", choices=sorted(LIDARS),
                    help="lidar model (config/lidars.yaml); default: the robot's (config/robots.yaml)")
parser.add_argument("--spawn", type=float, nargs="+", default=None, metavar="X Y [Z]",
                    help="start position [m]; without Z the drone stands on the floor below (x, y)")
parser.add_argument("--yaw", type=float, default=None, metavar="DEG", help="start heading [deg]")
parser.add_argument("--no_realtime", action="store_true", help="run as fast as possible instead of wall-clock pace")
parser.add_argument("--see_through", type=float, default=0.0, metavar="OPACITY",
                    help="draw the scene semi-transparent (0.25 works well) and let the GUI camera follow the drone")
parser.add_argument("--no_camera", action="store_true",
                    help="no front camera (faster: headless, nothing is rendered then)")
parser.add_argument("--light", choices=["dark", "bright"], default="dark",
                    help="dark: only the drone's LED lights the scene, as underground (default); bright: a dome light")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = not args.no_camera   # renders headless too (Isaac Lab's rendering experience)
app = AppLauncher(args).app

import math  # noqa: E402

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import torch  # noqa: E402
from geometry_msgs.msg import Twist, TwistStamped  # noqa: E402
from std_msgs.msg import String  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sensors import ContactSensorCfg, ImuCfg  # noqa: E402
from isaaclab.terrains import TerrainImporterCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

import sim_common as sc  # noqa: E402  (next to this script: scene, lidar, IMU and ground-truth output)
from rotors import LeeController, Rotors  # noqa: E402  (ported from RotorS, Apache-2.0)

R = sc.config("robots.yaml")[args.robot]
MODEL = R.get("model", args.robot)   # the airframe, shared by the lidar variants
USD = f"{sc.ROOT}/assets/drones/{MODEL}/{MODEL}.usd"
if not os.path.exists(USD):
    raise SystemExit(f"[drone_sim] no {USD}: run tools/add_drone.sh first")
args.lidar = args.lidar or R["lidar"]
LIDAR = LIDARS[args.lidar]
SCENE_USD, SCENE_NAME, SPAWN, YAW = sc.scene_and_start(args.scene, args.spawn, args.yaw, R["spawn_z"])
DT = 0.005                     # s, physics and control rate (200 Hz, as the legged robots)
CMD_LIMITS = np.array([2.0, 2.0, 1.0, 1.0])   # vx, vy, vz [m/s], yaw rate [rad/s]
CMD_SLEW = np.array([2.0, 2.0, 1.5, 2.0])     # m/s^2, rad/s^2: a smooth reference for the controller
CMD_TIMEOUT = 0.5              # s (sim) without a command -> hover
REF_LEASH = 1.0                # m: the reference stays this close to the drone (a blocked drone does not wind it up)
V_LOCK = 0.1                   # m/s: without a command and slower than this, the position is held (per axis group)
CRASH_TILT = math.radians(60.0)


@configclass
class DroneSceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(prim_path="/World/ground", terrain_type="usd", usd_path=SCENE_USD)
    drone = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Drone",
        spawn=sim_utils.UsdFileCfg(
            usd_path=USD, activate_contact_sensors=True,
            # no PhysX damping (the rotor drag is modelled) and no sleeping (a resting body would ignore the thrust)
            rigid_props=sim_utils.RigidBodyPropertiesCfg(linear_damping=0.0, angular_damping=0.0,
                                                         sleep_threshold=0.0, stabilization_threshold=0.0)),
        init_state=RigidObjectCfg.InitialStateCfg(pos=SPAWN, rot=(math.cos(YAW / 2), 0.0, 0.0, math.sin(YAW / 2))))
    imu = ImuCfg(prim_path="{ENV_REGEX_NS}/Drone")
    lidar = sc.lidar_cfg("{ENV_REGEX_NS}/Drone", LIDAR, R["lidar_pos"], SCENE_USD, DT)
    contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Drone")
    light = AssetBaseCfg(prim_path="/World/light",
                         spawn=sim_utils.DomeLightCfg(intensity=2000.0 if args.light == "bright" else 0.0))


sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, render_interval=4, device=args.device))
if args.light == "dark":   # Isaac Lab's rendering experience (apps/*rendering.kit) lights everything with an ambient
    import carb
    carb.settings.get_settings().set("/rtx/sceneDb/ambientLightIntensity", 0.0)
scene = InteractiveScene(DroneSceneCfg(num_envs=1, env_spacing=2.0))
if args.see_through > 0:
    sc.see_through(args.see_through)
sc.headlight("/World/envs/env_0/Drone/led", R["led"])
rclpy.init()
node = rclpy.create_node("drone_sim")
camera = None if args.no_camera else sc.CameraOut(node, "/World/envs/env_0/Drone/camera", R["camera"])
CAM_EVERY = round(1.0 / (R["camera"]["hz"] * DT))   # physics steps per camera frame
sim.reset()
drone, contact = scene["drone"], scene["contact"]
print(f"[drone_sim] {args.robot}: {R['mass']} kg, {len(R['rotors'])} rotors; lidar {sc.lidar_desc(args.lidar, LIDAR)}",
      flush=True)
print(f"[drone_sim] scene {SCENE_NAME}: start at ({SPAWN[0]:.1f}, {SPAWN[1]:.1f}, {SPAWN[2]:.2f}) heading "
      f"{math.degrees(YAW):.0f} deg", flush=True)

sensors = sc.Sensors(node, scene["imu"], scene["lidar"], drone, R["lidar_pos"], DT, LIDAR)
cmd = {"v": np.zeros(4), "t": -1e9}   # latest command (vx, vy, vz, yaw rate) and its sim time


def on_cmd(m):
    cmd["v"] = np.clip([m.linear.x, m.linear.y, m.linear.z, m.angular.z], -CMD_LIMITS, CMD_LIMITS)
    cmd["t"] = sim.current_time


node.create_subscription(Twist, "/cmd_vel", on_cmd, 10)
node.create_subscription(TwistStamped, "/cmd_vel_stamped", lambda m: on_cmd(m.twist), 10)
reset_req = []
node.create_subscription(String, "/sim/reset", lambda m: reset_req.append(m.data.strip() or "here"), 1)

lee, rotors = LeeController(R), Rotors(R)
st = {"armed": False, "crashed": False, "p_ref": np.zeros(3), "yaw_ref": 0.0, "applied": np.zeros(4),
      "airborne_t": 0.0, "collisions": 0}


def state():
    d = drone.data
    q = d.root_quat_w[0].tolist()   # w x y z
    w_, x, y, z = q
    rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w_), 2 * (x * z + y * w_)],
                    [2 * (x * y + z * w_), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w_)],
                    [2 * (x * z - y * w_), 2 * (y * z + x * w_), 1 - 2 * (x * x + y * y)]])
    return (np.array(d.root_pos_w[0].tolist()), np.array(d.root_lin_vel_w[0].tolist()), rot,
            np.array(d.root_ang_vel_b[0].tolist()))


def reset(mode):
    p, _, rot, _ = state()
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    if mode == "spawn":
        pos, yaw = np.array(SPAWN), YAW
    else:
        pos = p + np.array([0.0, 0.0, 0.2])
    pose = torch.tensor([[*pos, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]], device=sim.device)
    with torch.inference_mode():
        drone.write_root_pose_to_sim(pose)
        drone.write_root_velocity_to_sim(torch.zeros(1, 6, device=sim.device))
    rotors.speed[:] = 0.0
    st.update(armed=mode != "spawn", crashed=False, p_ref=pos.copy(), yaw_ref=yaw, applied=np.zeros(4))
    cmd["v"] = np.zeros(4)
    print(f"[drone_sim] reset ({mode}) to ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})"
          + ("" if mode == "spawn" else ", hovering") + ". Restart the LIO now.", flush=True)


def control(t):
    """One physics step of the flight controller: command -> reference -> rotor speeds -> wrench on the body."""
    p, v, rot, w = state()
    target = cmd["v"] if t - cmd["t"] < CMD_TIMEOUT else np.zeros(4)
    st["applied"] += np.clip(target - st["applied"], -CMD_SLEW * DT, CMD_SLEW * DT)
    if rot[2, 2] < math.cos(CRASH_TILT) and st["armed"]:
        st.update(armed=False, crashed=True)
        print(f"[drone_sim] CRASH at ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}): tilted "
              f"{math.degrees(math.acos(max(-1.0, rot[2, 2]))):.0f} deg, motors off. Reset with:  ros2 topic pub --once "
              "/sim/reset std_msgs/msg/String \"{data: here}\"   (or data: spawn)", flush=True)
    if not st["armed"] and not st["crashed"] and np.abs(target).max() > 1e-3:
        st.update(armed=True, p_ref=p.copy(), yaw_ref=math.atan2(rot[1, 0], rot[0, 0]))
        print(f"[drone_sim] motors on at ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})", flush=True)
    if st["armed"]:
        vx, vy, vz, wz = st["applied"]
        c, s = math.cos(st["yaw_ref"]), math.sin(st["yaw_ref"])
        v_ref = np.array([c * vx - s * vy, s * vx + c * vy, vz])
        # flying or braking: the reference is the drone itself (velocity control); stopped: it stays where it was set
        # (position hold). Horizontal and vertical apart, so the height is held while flying level.
        if math.hypot(vx, vy) > 1e-3 or math.hypot(v[0], v[1]) > V_LOCK:
            st["p_ref"][:2] = p[:2]
        if abs(vz) > 1e-3 or abs(v[2]) > V_LOCK:
            st["p_ref"][2] = p[2]
        st["yaw_ref"] += wz * DT
        d = st["p_ref"] - p
        if np.linalg.norm(d) > REF_LEASH:
            st["p_ref"] = p + d / np.linalg.norm(d) * REF_LEASH
        speed_cmd = lee.rotor_speeds(p, v, rot, w, st["p_ref"], v_ref, st["yaw_ref"], wz)
    else:
        speed_cmd = np.zeros(len(R["rotors"]))
    force, torque = rotors.step(speed_cmd, rot.T @ v, DT)
    drone.set_external_force_and_torque(torch.tensor(force, dtype=torch.float32, device=sim.device).view(1, 1, 3),
                                        torch.tensor(torque, dtype=torch.float32, device=sim.device).view(1, 1, 3))
    # collisions: contact after a contact-free second of flight (the start on the floor and landings are not one)
    touching = contact.data.net_forces_w[0].norm(dim=-1).max().item() > 0.5
    if st["armed"] and not touching:
        st["airborne_t"] += DT
    elif touching:
        if st["armed"] and st["airborne_t"] > 1.0:
            st["collisions"] += 1
            print(f"[drone_sim] collision #{st['collisions']} at ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}), "
                  f"speed {np.linalg.norm(v):.2f} m/s", flush=True)
        st["airborne_t"] = 0.0


gui = sim.has_gui()
for _ in range(int(0.5 / DT)):   # land on the floor before anything is published
    scene.write_data_to_sim()
    sim.step(render=False)
    scene.update(DT)
wall0, sim0, k = time.time(), sim.current_time, 0
print(f"[drone_sim] running: robot={args.robot} scene={SCENE_NAME}. Fly with /cmd_vel (sim/drone_test.py).", flush=True)
while app.is_running():
    rclpy.spin_once(node, timeout_sec=0.0)
    if reset_req:
        reset(reset_req.pop())
    t = sim.current_time
    control(t)
    scene.write_data_to_sim()
    k += 1
    sim.step(render=False)
    shot = camera is not None and k % CAM_EVERY == 0
    if (gui and k % 4 == 0) or shot:
        sim.render()
        if gui and args.see_through > 0:   # follow: behind and above the drone
            p, _, rot, _ = state()
            hd = rot[:2, 0] / max(np.linalg.norm(rot[:2, 0]), 1e-6)
            sim.set_camera_view(eye=[p[0] - 3 * hd[0], p[1] - 3 * hd[1], p[2] + 2.0], target=p.tolist())
    scene.update(DT)
    t = sim.current_time
    sensors.imu(t)
    sensors.lidar_step(t)
    if k % 4 == 0:
        sensors.gt(t)
    if shot:
        camera.publish(t, CAM_EVERY * DT)
    if not args.no_realtime:
        lag = (t - sim0) - (time.time() - wall0)
        if lag > 0:
            time.sleep(lag)

node.destroy_node()
rclpy.shutdown()
app.close()
