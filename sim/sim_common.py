"""Shared by the robot simulators (go2_sim.py: legged robots, drone_sim.py: multirotors): the scene and the start pose
on its floor, the lidar (Isaac Lab RayCaster swept like a spinning head, or re-aimed along a Livox rosette), the IMU + /clock and ground-truth odometry
publishers, the camera and its headlight, the see-through rendering. Import after the Isaac Sim app has started."""
import math
import os

import numpy as np
import torch
import yaml
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField

import isaaclab.sim as sim_utils
from isaaclab.sensors import Camera, CameraCfg, RayCasterCfg, patterns
from isaaclab.utils.math import quat_apply_inverse

ROOT = "/workspace/isaac_explore"
SCENE_MESH = "/World/ground/terrain/collision"  # sdf_to_usd.py bakes all colliders into this one mesh
LIDAR_MESH = "/World/ground/terrain/lidar"  # ... and all visual geometry into this one: what Gazebo's gpu_lidar sees
LIDAR_HZ = 10


def config(name):
    return yaml.safe_load(open(f"{ROOT}/config/{name}")) or {}


def floor_z(usd, x, y, z_ref):
    """Height of the scene collider at (x, y): of the surfaces above / below that point, the one closest to z_ref
    (not the highest: caves have a roof). The collider is one triangle mesh in world coordinates (sdf_to_usd.py)."""
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(usd)   # keep a reference: a temporary stage is freed and its prims turn invalid
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath("/World/collision"))
    p = np.array(mesh.GetPointsAttr().Get(), float)
    a, b, c = (p[i] for i in np.array(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3).T)
    e1, e2, q = b[:, :2] - a[:, :2], c[:, :2] - a[:, :2], np.array([x, y]) - a[:, :2]
    den = e1[:, 0] * e2[:, 1] - e2[:, 0] * e1[:, 1]
    den = np.where(np.abs(den) > 1e-12, den, np.nan)   # vertical triangles have no xy area
    u = (q[:, 0] * e2[:, 1] - e2[:, 0] * q[:, 1]) / den   # barycentric coordinates in the xy projection
    v = (e1[:, 0] * q[:, 1] - q[:, 0] * e1[:, 1]) / den
    hit = (u >= 0) & (v >= 0) & (u + v <= 1)
    z = (a[:, 2] + u * (b[:, 2] - a[:, 2]) + v * (c[:, 2] - a[:, 2]))[hit]
    return float(z[np.argmin(np.abs(z - z_ref))]) if len(z) else z_ref


def scene_and_start(scene, spawn, yaw, height):
    """--scene/--spawn/--yaw -> (scene usd, scene name, start position, start yaw [rad]). Defaults: config/scenes.yaml
    (spawn X Y and the reference height that picks the floor), else (2, 0); the start height is `height` above the
    floor below (X, Y) unless --spawn gives Z."""
    usd = scene if scene.endswith(".usd") else f"{ROOT}/sim/scenes/{scene}.usd"
    name = os.path.splitext(os.path.basename(usd))[0]
    s = config("scenes.yaml").get(name, {})
    x, y, z_ref = (list(s.get("spawn", [2.0, 0.0])) + [0.0])[:3]
    if spawn:
        x, y = spawn[:2]
    pos = tuple(spawn[:3]) if spawn and len(spawn) >= 3 else (x, y, floor_z(usd, x, y, z_ref) + height)
    return usd, name, pos, math.radians(yaw if yaw is not None else s.get("yaw", 0.0))


def rosette(lidar, shots):
    """Unit ray directions (n, 3) of the shots (indices, at lidar["rate"] per second from 0) of a Livox lidar of
    config/lidars.yaml: two prisms turning in opposite directions (Risley prisms, first-order deflection) draw a
    rosette over the unit disk, stretched to hfov x vfov around the mount's +x. Its petals precess, so the field of
    view fills in over time instead of repeating each frame (the rosette of Livox's lidars, e.g. Lin & Zhang,
    "Loam_livox", ICRA 2020, arXiv:1909.06700). The real prism speeds and laser layout are not published."""
    t = shots.double() / lidar["rate"]
    a, b = 2 * math.pi * lidar["prisms_hz"][0] * t, -2 * math.pi * lidar["prisms_hz"][1] * t
    x, y = (torch.cos(a) + torch.cos(b)) / 2, (torch.sin(a) + torch.sin(b)) / 2
    lo, hi = math.radians(lidar["vfov"][0]), math.radians(lidar["vfov"][1])
    az, el = x * math.radians(lidar["hfov"]) / 2, (lo + hi) / 2 + y * (hi - lo) / 2
    return torch.stack([torch.cos(el) * torch.cos(az), torch.cos(el) * torch.sin(az), torch.sin(el)], -1).float()


def lidar_desc(name, lidar):
    if lidar.get("type") == "livox":
        return (f"{name}: Livox rosette {lidar['hfov']} x {lidar['vfov'][1] - lidar['vfov'][0]:.1f} deg, "
                f"{lidar['rate']} points/s, {lidar['range']} m")
    return (f"{name}: {lidar['channels']} beams {lidar['vfov'][0]}..{lidar['vfov'][1]} deg, {lidar['cols']} columns, "
            f"{lidar['range']} m")


def lidar_cfg(prim_path, lidar, lidar_pos, usd, physics_dt):
    """RayCaster for a lidar of config/lidars.yaml mounted at lidar_pos on prim_path (no rotation). It ray-casts the
    visual mesh of the scene usd, or its collider in scenes without one (mesh_to_usd.py, conversions before v3.0.0).
    A Livox lidar gets the rays of one physics step (Sensors re-aims them every step); a spinning one all its beams."""
    from pxr import Sdf
    layer = Sdf.Layer.FindOrOpen(usd)  # keep a reference: a temporary layer is freed and its prim specs turn dormant
    mesh = LIDAR_MESH if layer.GetPrimAtPath("/World/lidar") else SCENE_MESH
    print(f"[sim] lidar ray-casts {mesh}", flush=True)
    if lidar.get("type") == "livox":
        n = round(lidar["rate"] * physics_dt)
        pattern = patterns.PatternBaseCfg(
            func=lambda cfg, device: (torch.zeros(n, 3, device=device), rosette(lidar, torch.arange(n)).to(device)))
    else:
        pattern = patterns.LidarPatternCfg(channels=lidar["channels"], vertical_fov_range=tuple(lidar["vfov"]),
                                           horizontal_fov_range=(-180.0, 180.0), horizontal_res=360.0 / lidar["cols"])
    return RayCasterCfg(prim_path=prim_path, mesh_prim_paths=[mesh], max_distance=lidar["range"],
                        offset=RayCasterCfg.OffsetCfg(pos=tuple(lidar_pos)), ray_alignment="base", pattern_cfg=pattern)


def stamp(t):
    ns = int(round(t * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


PC_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("t", "<u4")])
PC_FIELDS = [PointField(name=n, offset=PC_DTYPE.fields[n][1], datatype=dt, count=1)
             for n, dt in (("x", 7), ("y", 7), ("z", 7), ("intensity", 7), ("t", 6))]


class Sensors:
    """ROS 2 output of the robot's sensors: /imu/data + /clock (call imu() every physics step), /lidar/points (call
    lidar_step() every physics step: a rolling scan), /ground_truth/odom (gt()). `body` is the asset whose pose is the
    ground truth (robot base = imu frame); `spec`: the lidar's entry in config/lidars.yaml."""

    def __init__(self, node, imu, lidar, body, lidar_pos, physics_dt, spec):
        self.imu_sensor, self.lidar, self.body = imu, lidar, body
        self.pub_clock = node.create_publisher(Clock, "/clock", 10)
        self.pub_imu = node.create_publisher(Imu, "/imu/data", 200)
        self.pub_pc = node.create_publisher(PointCloud2, "/lidar/points", 5)
        self.pub_gt = node.create_publisher(Odometry, "/ground_truth/odom", 10)
        # rolling scan: each physics step casts only the azimuth slice the spinning head is at, in the sensor frame
        # of that instant (exactly how a real spinning lidar distorts under motion)
        self.n_slices = round(1.0 / (LIDAR_HZ * physics_dt))  # 20 at 200 Hz physics
        self.livox = spec if spec.get("type") == "livox" else None
        if self.livox:   # all rays are this step's shots of the rosette (lidar_cfg); re-aimed after every cast
            self.slice_rays, self.shot = [slice(None)] * self.n_slices, lidar.num_rays
        else:
            az = torch.atan2(lidar.ray_directions[0, :, 1], lidar.ray_directions[0, :, 0])  # [-pi, pi)
            k = ((az + math.pi) / (2 * math.pi) * self.n_slices).long().clamp(max=self.n_slices - 1)
            self.slice_rays = [torch.nonzero(k == i)[:, 0] for i in range(self.n_slices)]
        self.scan = {"k": 0, "t0": 0.0, "pts": [], "ts": []}
        self.lidar_offset = torch.tensor(lidar_pos, device=lidar.device, dtype=torch.float32)  # mount: no rotation

    def imu(self, t):
        d = self.imu_sensor.data
        m = Imu()
        m.header.stamp, m.header.frame_id = stamp(t), "imu"
        w, a, q = d.ang_vel_b[0].tolist(), d.lin_acc_b[0].tolist(), d.quat_w[0].tolist()  # quat wxyz
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = w
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = a
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = q
        self.pub_imu.publish(m)
        c = Clock()
        c.clock = m.header.stamp
        self.pub_clock.publish(c)

    def gt(self, t):
        r = self.body.data
        m = Odometry()
        m.header.stamp, m.header.frame_id, m.child_frame_id = stamp(t), "world", "base_gt"
        p, q = r.root_pos_w[0].tolist(), r.root_quat_w[0].tolist()
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = p
        m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z = q
        v, w = r.root_lin_vel_b[0].tolist(), r.root_ang_vel_b[0].tolist()
        m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = v
        m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z = w
        self.pub_gt.publish(m)

    def lidar_step(self, t):
        """Cast the current slice; publish when the revolution completes."""
        s = self.scan
        k = s["k"]
        if k == 0:
            s["t0"], s["pts"], s["ts"] = t, [], []
        d = self.lidar.data
        hits = d.ray_hits_w[0, self.slice_rays[k]]
        hits = hits[torch.isfinite(hits).all(-1)]
        # RayCaster's data.pos_w/quat_w are the *base* pose (the mount offset is only baked into the ray starts), so
        # subtract the mount to get points in the lidar frame. Without this the cloud was 0.18 m off the extrinsic in
        # config/bievr_sim_go2.yaml: invisible when walking straight, but every in-place turn made BIEVR-LIO diverge.
        p = quat_apply_inverse(d.quat_w[0].expand(len(hits), 4), hits - d.pos_w[0]) - self.lidar_offset
        s["pts"].append(p)
        s["ts"].append(torch.full((len(p),), t - s["t0"], device=p.device))
        if self.livox:   # aim at the next shots (a new tensor: RayCaster's own may be an inference tensor)
            n = self.lidar.num_rays
            self.lidar.ray_directions = rosette(self.livox, torch.arange(self.shot, self.shot + n)).to(p.device)[None]
            self.shot += n
        s["k"] = (k + 1) % self.n_slices
        if s["k"] == 0:
            self._publish_scan(torch.cat(s["pts"]).cpu().numpy(), torch.cat(s["ts"]).cpu().numpy(), s["t0"])

    def _publish_scan(self, xyz, dt, t0):
        pc = np.zeros(len(xyz), PC_DTYPE)
        pc["x"], pc["y"], pc["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        pc["t"] = np.round(dt * 1e9).astype(np.uint32)
        m = PointCloud2()
        m.header.stamp, m.header.frame_id = stamp(t0), "lidar"  # Ouster convention: stamp = scan start
        m.height, m.width = 1, len(pc)
        m.fields, m.is_bigendian, m.point_step = PC_FIELDS, False, PC_DTYPE.itemsize
        m.row_step, m.is_dense, m.data = PC_DTYPE.itemsize * len(pc), True, pc.tobytes()
        self.pub_pc.publish(m)


def camera_k(c):
    """Intrinsic matrix (row-major 3x3) of a robot's `camera`: pinhole, square pixels, horizontal field of view hfov."""
    w, h = c["width"], c["height"]
    f = w / 2 / math.tan(math.radians(c["hfov"]) / 2)
    return [f, 0.0, w / 2, 0.0, f, h / 2, 0.0, 0.0, 1.0]


def camera_cfg(prim_path, c, pos, data_types=("rgb",), **kw):
    """Isaac Lab camera of a robot's `camera` at pos on prim_path, looking along its +x; kw: more CameraCfg fields."""
    return CameraCfg(
        prim_path=prim_path, width=c["width"], height=c["height"], data_types=list(data_types), update_period=0.0,
        spawn=sim_utils.PinholeCameraCfg.from_intrinsic_matrix(camera_k(c), c["width"], c["height"],
                                                               clipping_range=(0.05, 200.0)),
        offset=CameraCfg.OffsetCfg(pos=tuple(pos), rot=(1.0, 0.0, 0.0, 0.0), convention="world"), **kw)


class CameraOut:
    """A robot's `camera` (config/robots.yaml: pinhole with square pixels and the horizontal field of view hfov, at pos
    on prim_path looking along its +x) -> /camera/image_raw (rgb8) + /camera/camera_info, frame camera (ROS optical
    frame: z forward, x right, y down; explore.launch.py publishes imu -> camera). Not part of the scene: the simulator
    renders at the camera rate and calls publish() right after that render."""

    def __init__(self, node, prim_path, c):
        w, h, k = c["width"], c["height"], camera_k(c)
        self.info = CameraInfo(width=w, height=h, distortion_model="plumb_bob", d=[0.0] * 5, k=k,
                               r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                               p=[k[0], 0.0, k[2], 0.0, 0.0, k[4], k[5], 0.0, 0.0, 0.0, 1.0, 0.0])
        self.cam = Camera(camera_cfg(prim_path, c, c["pos"]))
        self.pub_img = node.create_publisher(Image, "/camera/image_raw", 2)
        self.pub_info = node.create_publisher(CameraInfo, "/camera/camera_info", 2)

    def publish(self, t, dt):
        """dt: time since the last publish (the camera period)."""
        self.cam.update(dt)
        rgb = self.cam.data.output["rgb"][0, ..., :3].contiguous().cpu().numpy()
        m = Image(height=rgb.shape[0], width=rgb.shape[1], encoding="rgb8", step=rgb.shape[1] * 3, data=rgb.tobytes())
        m.header.stamp, m.header.frame_id = stamp(t), "camera"
        self.info.header = m.header
        self.pub_img.publish(m)
        self.pub_info.publish(self.info)


def headlight(path, led):
    """A robot's `led` (config/robots.yaml): a disk light at pos under the robot prim path, shining along the robot's +x
    in a cone of `cone` degrees (full angle)."""
    import omni.usd
    from pxr import Gf, UsdGeom, UsdLux
    light = UsdLux.DiskLight.Define(omni.usd.get_context().get_stage(), path)
    light.CreateIntensityAttr(led["intensity"])
    light.CreateRadiusAttr(0.02)
    shaping = UsdLux.ShapingAPI.Apply(light.GetPrim())
    shaping.CreateShapingConeAngleAttr(led["cone"] / 2)
    shaping.CreateShapingConeSoftnessAttr(0.3)
    xf = UsdGeom.Xformable(light)
    xf.AddTranslateOp().Set(Gf.Vec3d(*led["pos"]))
    xf.AddOrientOp().Set(Gf.Quatf(math.sqrt(0.5), 0.0, -math.sqrt(0.5), 0.0))   # a disk light emits along its -z


def see_through(opacity):
    """One translucent material over the whole scene subtree, overriding the meshes' own materials (robots live under
    /World/envs and keep their look). Isaac Lab's render presets switch RTX translucency off; without it any
    opacity < 1 renders the rock fully invisible. Dark blue stays visible against the bright sky."""
    import carb

    carb.settings.get_settings().set("/rtx/translucency/enabled", True)
    carb.settings.get_settings().set("/rtx/raytracing/fractionalCutoutOpacity", True)
    mat = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.15, 0.3, 0.6), roughness=1.0, opacity=opacity)
    mat.func("/World/Looks/see_through", mat)
    sim_utils.bind_visual_material("/World/ground", "/World/Looks/see_through", stronger_than_descendants=True)
