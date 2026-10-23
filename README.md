# Terra Incognita

**English** | [中文](README.zh-CN.md)

**A simulation platform for autonomous exploration of unknown underground environments with legged robots.** Built on Isaac Sim and ROS 2: one script installs everything, and two terminals run the full closed loop of localization and mapping, autonomous exploration and locomotion. It is meant to get a simulation set up quickly at the start of a research project, so that you can then swap one of the modules for your own algorithm.

<p align="center">
  <img src="media/isaac_sim.png" width="49%" alt="Isaac Sim: ANYmal D in the starting area of the SubT cave (see-through rendering)">
  <img src="media/rviz.png" width="49%" alt="RViz: localization, mapping and TARE exploration planning">
  <br><sub>TARE in the SubT cave <code>simple_cave_01</code>: Isaac Sim (left) and RViz (right)</sub>
</p>
<p align="center">
  <img src="media/gbplanner_ros2_rviz.png" width="49%" alt="RViz: point-cloud map, the robot's trajectory (yellow) and the current way point (purple) while GBPlanner2 explores">
  <img src="media/gbplanner_rviz.png" width="49%" alt="GBPlanner2's RViz: occupancy voxel map coloured by height and the planner's graph">
  <br><sub>GBPlanner2 in <code>simple_cave_02</code>: RViz (left) and GBPlanner2's own RViz with its voxel map and planning graph (right)</sub>
</p>

## Features

- **Scenes**: the DARPA SubT cave `simple_cave_01` (installed by `setup.sh`), and one command away 33 more: the other SubT caves, mine tunnels, NIOSH mine replicas, finals and urban worlds, real mine scans, procedurally generated confined terrain (see [More scenes](#more-scenes)).
- **Robots**: ANYmal D (default) and Unitree Go2, driven by Isaac Lab's official pretrained rough-terrain walking policies, which take velocity commands.
- **Sensors**: simulated spinning lidar (10 Hz, per-point timestamps and real motion distortion), Ouster OS1-32 by default or one of 6 other models (`--lidar`: Ouster OS1-64, OS1-128, OS0-128, Velodyne VLP-16, HDL-32E, Hesai PandarXT-32), 200 Hz IMU, ground-truth pose.
- **Algorithms**:
  - [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO): lidar-inertial localization and mapping;
  - [elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy): GPU elevation map;
  - CMU's [terrain analysis, local planner and path follower](https://github.com/HongbiaoZ/autonomous_exploration_development_environment);
  - [TARE](https://github.com/caochao39/tare_planner): exploration planner;
  - optional: [GBPlanner2](https://github.com/ntnu-arl/gbplanner_ros/tree/gbplanner2) instead of TARE, as a baseline for comparing exploration strategies (see [GBPlanner2](#gbplanner2-optional-experimental)).
- **One Docker image** holds the whole environment (Isaac Sim 4.5, Isaac Lab 2.1.1, ROS 2 Humble, the algorithms' dependencies); all third-party sources are pinned, so the setup is reproducible.
- **Easy to swap**: any layer, such as localization or exploration planning, can be replaced by your own node (see [Plugging in your own algorithm](#plugging-in-your-own-algorithm)).

```
┌──────────────────────── Isaac Sim + Isaac Lab (sim.sh) ────────────────────────┐
│  physics ─ walking policy (velocity command -> joints) ─ lidar / IMU / truth   │
└──────┬───────────────────────────────────────────────────────────▲─────────────┘
       │ /lidar/points  /imu/data                                  │ /cmd_vel_stamped
┌──────▼───────────────────────── ROS 2 algorithms (explore.sh) ───┴─────────────┐
│ BIEVR-LIO ─► terrain analysis ─► TARE exploration ─► local planner ─► follower │
│           └─► elevation map                                                    │
└────────────────────────────────────────────────────────────────────────────────┘
```

## Requirements

| Item | Requirement | Tested with |
|---|---|---|
| OS | Linux x86_64 (Ubuntu 20.04 / 22.04) | Ubuntu 20.04 |
| GPU | NVIDIA RTX, ≥ 8 GB VRAM | RTX 4080 SUPER 16 GB |
| Driver | ≥ 535.129 (required by Isaac Sim 4.5) | 550 |
| RAM | 32 GB (the first Isaac Sim start compiles the render pipelines and peaks at ~21 GB; later ~5 GB) | 32 GB |
| Disk | ~70 GB free on the Docker partition | |
| Network | ~30 GB to download on the first installation | |

The lidar is simulated with ray casting, so Isaac Sim's RTX lidar (which needs driver 570 or newer) is not needed.

## Installation

### Step 0: prepare the host

You need the NVIDIA driver, Docker and the NVIDIA Container Toolkit. Skip what you already have; the first step of `setup.sh` checks them too.

```bash
nvidia-smi                                   # should show your GPU and a driver version >= 535

# Docker (official install script); let your user run docker without sudo (log in again afterwards)
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER

# NVIDIA Container Toolkit (see https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker

docker run --rm --gpus all ubuntu nvidia-smi  # check: the GPU is visible inside a container
```

### Step 1: get the project

```bash
git clone https://github.com/ygd551/terra-incognita.git
cd terra-incognita
```

### Step 2: install everything

```bash
./setup.sh          # in mainland China: ./setup.sh --cn (Tsinghua apt mirrors for the image build)
```

`setup.sh` goes through the 7 steps below. **If it fails halfway (e.g. a network drop), just run it again**: finished steps are skipped.

| Step | What it does | First run (approx.) |
|---|---|---|
| 1 | Checks the driver, Docker, the NVIDIA Container Toolkit and the disk space | – |
| 2 | Fetches the third-party ROS 2 sources at pinned versions (BIEVR-LIO, elevation_mapping_cupy, TARE, CMU environment) and applies this project's patches | a few minutes |
| 3 | Clones and patches Isaac Lab v2.1.1, builds the Isaac Sim + Isaac Lab base image | 1–2 hours |
| 4 | Builds the project image `isaac_explore`: ROS 2 Humble and the algorithms' dependencies | ~40 minutes |
| 5 | Downloads the walking-policy checkpoints (published by Isaac Lab) and the SubT cave (~100 MB) | a few minutes |
| 6 | Converts the scene from Gazebo SDF to Isaac Sim USD | a few minutes |
| 7 | Builds the ROS 2 workspace | ~2 minutes |

It ends with `Installation complete` when everything is ready.

Step 6 is the first Isaac Sim start, which compiles the render pipelines with a memory peak of ~21 GB, only once. With 32 GB of RAM, close memory-hungry programs such as browsers during the installation. Everything runs in one container with a memory cap (by default total RAM minus 9 GB), so even if that is exceeded, only the processes in the container are killed and the desktop does not freeze.

- If you already have Isaac Lab v2.1.1: `ISAACLAB_DIR=/path/to/IsaacLab ./setup.sh` uses it instead of cloning another copy.
- If `docker pull` of the Isaac Sim image (15 GB) keeps failing, the script falls back to a resumable download (`tools/fetch_image.sh`).
- If Docker needs a proxy: set it for the Docker daemon (`/etc/systemd/system/docker.service.d/http-proxy.conf`) for pulls, and under `proxies` in `~/.docker/config.json` for builds.

**Updating**: `git pull && ./setup.sh` rebuilds images and recompiles as needed; unchanged steps take seconds.

## Running

Open two terminals, both in the project directory.

**Terminal 1: the simulation**

```bash
./sim.sh
```

The first start takes a few minutes (shader compilation, robot model download). The Isaac Sim window opens and ANYmal D stands at the cave entrance; wait until the terminal prints `[go2_sim] running` before going on.

**Terminal 2: the algorithms**

```bash
./explore.sh
```

RViz opens after a few seconds. Once the terminal shows `Exploration Started`, the robot explores on its own: it decides where to go, walks into the cave and avoids the rocks. When exploration is finished, it prints `Exploration completed, returning home` and walks back to the start.

**Stop**: run `./stop_all.sh` in any terminal.

### What you see

**Isaac Sim window**: the cave is closed, so the robot inside cannot be seen from outside. Start with `./sim.sh --see_through 0.25` to draw the cave semi-transparent, with the camera following the robot from above (picture above). Deeper in the cave there are more rock layers and the view gets darker; a smaller value (e.g. 0.1, smaller is more transparent) helps. This option only affects rendering, not physics or sensors. The exploration itself is easiest to follow in RViz.

**RViz window**, main symbols:

| Symbol | Meaning |
|---|---|
| Blue point cloud | Explored area (accumulated registered lidar points) |
| Short red and green lines | Robot pose: red is forward, green is left |
| Large green square | TARE's local planning horizon (30 m × 30 m); viewpoints are chosen only inside it |
| Small green squares | Subspaces still to explore, in visiting order (more solid = sooner). Exploration ends when they are all gone |
| Light cyan line / dark blue line | Global route / local route |
| Orange points | Selected viewpoints: places the robot will go to "take a look" |
| Red points | Surfaces not yet seen well and frontiers (boundary between known and unknown space) |
| Purple point | Current waypoint: ~5 m ahead, the target sent to the local planner |
| Bright green line in front of the robot | Path chosen by the local planner (~4 m, avoids obstacles) |
| Yellow points | Trajectory: where the robot has walked |

**Exploration metrics**: every run writes `logs/metrics_<time>.txt`, whose 4 columns are **explored volume (m³), distance travelled (m), algorithm runtime, elapsed time (s)**; `trajectory_<time>.txt` next to it holds the robot's trajectory.

## Usage

### Options

| Command | Option | Effect |
|---|---|---|
| `./sim.sh` | `--robot go2` | Use the Unitree Go2 (default ANYmal D). `explore.sh` automatically uses the same robot |
| | `--lidar vlp16` | Another lidar model (default `os1_32`): `os1_64`, `os1_128`, `os0_128`, `vlp16`, `hdl32`, `xt32`; beams, field of view, resolution and range in `config/lidars.yaml`, which also takes new models. Same message format for all; GBPlanner2's sensor model follows the vertical field of view. Not emulated: non-repetitive scan patterns (Livox, Unitree L1), intensity, noise |
| | `--headless` | No simulation window (saves GPU and RAM); watch in RViz only |
| | `--scene NAME` | Another scene (`./sim.sh --list`; see [More scenes](#more-scenes)). Its start pose comes from `config/scenes.yaml` |
| | `--spawn X Y [Z]` `--yaw DEG` | Another start pose; without Z the robot is put on the floor below (X, Y) |
| | `--see_through 0.25` | Semi-transparent scene, camera follows the robot |
| | `--auto_reset 10` | Stand the robot back up 10 s after a fall (for unattended runs with `lio:=gt`) |
| | `--help` | All options |
| `./explore.sh` | `lio:=gt` | Ground-truth localization instead of BIEVR-LIO, to test planning on its own |
| | `speed:=0.5` | Top speed, default 1.0 m/s (the limit the walking policies were trained for; do not go higher) |
| | `exploration:=false` | Everything except TARE: publish your own `/way_point` |
| | `exploration:=gbplanner` | GBPlanner2 explores instead of TARE (once installed, see [GBPlanner2](#gbplanner2-optional-experimental)) |
| | `planner:=false` | Localization and mapping only (no exploration, no local planner) |
| | `rviz:=false`, `elevation:=false` | No RViz, no elevation map |

### Common tasks

```bash
# The robot fell (the sim terminal prints "robot has fallen"): here = stand up in place, spawn = back to the start
docker exec isaac_explore bash -c 'source /opt/ros/humble/setup.bash && ros2 topic pub --once /sim/reset std_msgs/msg/String "{data: here}"'
#   the pose jumps, which BIEVR cannot follow; with BIEVR it is simpler to ./stop_all.sh and start again

# A shell in the container, for ros2 commands or the project's tools
./container.sh
source /isaac_explore/docker/algo_env.sh
ros2 topic list
python3 /isaac_explore/sim/watch_explore.py 600       # live position, distance travelled, waypoint
python3 /isaac_explore/sim/eval_lio.py                # walk a fixed route and measure BIEVR-LIO against ground truth
python3 /isaac_explore/sim/drive_test.py 0.5 0 0 10   # send a velocity command: vx vy wz seconds (stop explore.sh first)

# One unattended trial (headless sim + all algorithms + recording); results in logs/trial_<name>/
LAUNCH_ARGS="lio:=gt" ./run_trial.sh NAME [MAX_SIM_SECONDS] [sim.sh options, e.g. --scene simple_tunnel_01]
```

### Changing parameters

Restart the corresponding terminal after a change. Config and launch files are read straight from the project directory, so nothing needs to be rebuilt.

| What | Where | How it takes effect |
|---|---|---|
| Top speed | `./explore.sh speed:=0.5` | restart terminal 2 |
| Robot, scene, start pose | `./sim.sh --robot / --scene / --spawn / --yaw`; default start pose of each scene: `config/scenes.yaml` | restart both terminals |
| Lidar model; its beams, field of view, resolution, range | `./sim.sh --lidar`; `config/lidars.yaml` | restart both terminals |
| Lidar mount, footprint, obstacle height threshold, viewpoint height, spawn height | `config/robots.yaml` | restart both terminals. The lidar mount must also be changed in `calibration` of `config/bievr_sim_<robot>.yaml` |
| BIEVR-LIO | `config/bievr_params.yaml`, `config/bievr_sim_<robot>.yaml` | restart terminal 2 |
| TARE exploration | `config/tare_go2.yaml` (shared by both robots) | restart terminal 2 |
| Terrain analysis, local planner, path follower (turn rate, acceleration, ...) | `launch/cmu/*.launch` | restart terminal 2 |
| Elevation map | `config/elevation_sim.yaml` | restart terminal 2 |
| Simulator-side speed limits, slowing down in turns, stuck recovery, fall detection | constants at the top of `sim/go2_sim.py` | restart terminal 1 |
| Source code of a third-party package | `ros2_ws/src/<package>/` | `./container.sh build`, then restart terminal 2. To keep a change, turn it into a patch in `ros2_ws/patches/` and register it in `ros2_ws/fetch_src.sh` |
| Container memory cap (default: total RAM minus 9 GB, between 16 and 26 GB) | `ISAAC_EXPLORE_MEM=24g ./container.sh rm`, then start as usual | recreates the container |

### More scenes

**Other SubT worlds and sample mines.** `tools/add_scenes.sh NAME...` downloads a world with the models it uses and converts it into `sim/scenes/NAME.usd`; `./sim.sh --list` shows what is there, `./sim.sh --scene NAME` starts it at its default start pose (`config/scenes.yaml`):

```bash
tools/add_scenes.sh simple_tunnel_01 niosh_sr_config_a   # names: osrf/subt worlds, or assets/subt/worlds/NAME.sdf
./sim.sh --list
./sim.sh --scene simple_tunnel_01
tools/check_scenes.sh simple_tunnel_01                  # optional: does the robot stand at the start pose?
```

| Kind | Scenes (all [osrf/subt](https://github.com/osrf/subt/tree/master/subt_ign/worlds) worlds have a tile graph for coverage) |
|---|---|
| Natural caves | `simple_cave_01..03`, `cave_qual`, `cave_circuit_practice_01..03` |
| Mine tunnels | `simple_tunnel_01..03`, `tunnel_qual_ign`, `tunnel_circuit_practice_01..03` |
| Mine replicas (NIOSH) | `niosh_sr_config_a/b`, `niosh_ex_config_a/b` |
| Mixed (finals) | `finals_practice_01..03`, `finals_qual` |
| Urban underground | `simple_urban_01..03`, `urban_circuit_practice_01..03` |
| [gbplanner_ros](https://github.com/ntnu-arl/gbplanner_ros) samples (`ros1_ws/add_gbp_scenes.sh`) | `gbp_darpa_cave_01/02` (same layouts as `simple_cave_01/02`, older tiles), `gbp_edgar_mine`, `gbp_pittsburgh_mine`, `gbp_virginia_mine` (real mine scans), `gbp_urban_cave_transition` |

All 33 extra scenes take ~2 GB of downloads and ~4 GB of disk; converting one takes 15–60 s. Sources and licenses of all these scenes: [License](#license). The SubT worlds start in the competition's staging area, 8 m before the course entrance; a new world gets a line in `config/scenes.yaml` (`tools/scene_spawn.py NAME X Y` finds a free floor point). SubT artifacts (backpacks, survivor mannequins, phones, ...) are left out of the scene by default; see `sim/sdf_to_usd.py --skip`.

**Procedurally generated confined terrain** ([leggedrobotics/terrain-generator](https://github.com/leggedrobotics/terrain-generator)): walls, steps, platforms and overhangs assembled into 40 m × 40 m maps, good for narrow, multi-level collapsed spaces. Needs a Python on the host with torch and open3d:

```bash
PY=/path/to/python terrain_gen/setup.sh        # first time: pinned sources, patch, virtual environment
terrain_gen/.venv/bin/python terrain_gen/generate.py --cfg indoor --enable_sdf --mesh_dir $PWD/terrain_gen/out/indoor
#   --cfg: indoor | overhanging | overhanging_floor; 10 maps per run, ~5 minutes
rm -f terrain_gen/out/*/*/{sdf,mesh_terrain_sdf,mesh_terrain_distance}.npy   # distance fields the sim does not use, ~470 MB per map
docker exec isaac_explore bash -c 'cd /workspace/isaac_explore && /isaac-sim/python.sh sim/mesh_to_usd.py \
  terrain_gen/out/indoor/mesh_5/mesh.obj sim/scenes/tg_indoor_5.usd --spawn terrain_gen/out/indoor/mesh_5/spawnable_locations.npy'
#   the last line suggests --spawn X Y Z (for the Go2's standing height; add ~0.23 m to Z for ANYmal D)
./sim.sh --scene tg_indoor_5 --spawn X Y Z           # or add its start pose to config/scenes.yaml
```

## GBPlanner2 (optional, experimental)

[GBPlanner2](https://github.com/ntnu-arl/gbplanner_ros/tree/gbplanner2) (graph-based exploration planner of NTNU's Autonomous Robots Lab, used by team CERBERUS, winner of the DARPA Subterranean Challenge) can explore instead of TARE, as a baseline for comparing exploration strategies. It exists for ROS 1 Noetic only, so it runs in a container of its own (image `gbplanner_ros1`) and talks to the ROS 2 stack through `ros1_bridge` (image `ros:foxy-ros1-bridge`). Only the odometry, the registered scan and `/clock` cross the bridge; an adapter node (`ros1_ws/scripts/isaac_adapter.py`) follows GBPlanner's path by sending way points ~5 m ahead to the CMU local planner, as TARE does. The integration is kept minimal: GBPlanner2 at a pinned commit with two guards against an upstream crash, its parameters those of its ground-robot sample (SMB) adapted to ANYmal D and the OS1-32 lidar (`ros1_ws/config/`).

**Install** (once; ~30 minutes, ~7.5 GB of images):

```bash
ros1_ws/fetch_src.sh            # GBPlanner2 and its dependencies at pinned commits, plus our patch
docker build --network host -t gbplanner_ros1 -f ros1_ws/docker/Dockerfile ros1_ws
#   in mainland China add: --build-arg UBUNTU_MIRROR=mirrors.tuna.tsinghua.edu.cn
#                          --build-arg ROS_APT_MIRROR=http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu
docker pull ros:foxy-ros1-bridge
ros1_ws/ros1.sh build           # catkin build inside the container, ~2 minutes
```

**Run**: terminal 1 `./sim.sh` as usual; terminal 2 `./explore.sh exploration:=gbplanner` (Ctrl-C stops GBPlanner too). The robot first walks 4 m straight ahead: the lidar cannot see the ground within ~2 m of the robot, and GBPlanner only plans over ground it has seen. A second RViz window shows GBPlanner's own view: local graph, global graph with its frontiers (red), the chosen path, the robot's history. Logs: `docker exec gbplanner_ros1 tail -f /tmp/gbplanner_launch.log /tmp/isaac_adapter.log`.

**Tools for comparisons and tests** (in `ros1_ws/`):

| Tool | What it does |
|---|---|
| `compare.sh [N] [SECONDS]` | N unattended headless trials each of TARE and GBPlanner2 under identical conditions, then the coverage of every trial (`logs/compare_<time>.txt`) |
| `run_trial.sh NAME [SECONDS] [sim.sh options]` | one unattended GBPlanner2 trial, like `./run_trial.sh` |
| `scripts/tile_coverage.py logs/trial_X/run.csv [metrics.txt]` | SubT tiles entered, distance and explored volume over sim time (any SubT world, `--world NAME`) |
| `ramp_test.sh`, `scripts/cmu_slope_test.py` | how steep the walking policy climbs and descends; from which slope the CMU terrain analysis reports obstacles |
| `add_gbp_scenes.sh` | the gbplanner_ros sample worlds as scenes (`gbp_*`, see [More scenes](#more-scenes)) |

**Known behaviour**: GBPlanner plans on its own voxblox map, the CMU local planner moves on the terrain analysis. Where they disagree, mostly on slopes steeper than ~20° (see [Known limitations](#known-limitations)), the robot goes back and forth until the adapter gives the path up after 20 s without progress and GBPlanner plans again. GBPlanner prefers large unknown volume, so it tends to explore open caverns before narrow tunnels.

## Plugging in your own algorithm

All modules talk over ROS 2 topics (`use_sim_time`, with the `/clock` published by the simulation). To replace a layer, switch it off and let your node subscribe to the topics upstream of it and publish the ones downstream:

| Module to replace | How to switch it off | Your node subscribes to | Your node publishes |
|---|---|---|---|
| Exploration planner (TARE) | `./explore.sh exploration:=false` | `/state_estimation`, `/registered_scan`, `/terrain_map`, `/terrain_map_ext` | `/way_point` (`geometry_msgs/PointStamped`, frame `map`, a goal a few metres ahead) |
| Local planner and control | `./explore.sh planner:=false` | as above, plus your own goal | `/cmd_vel_stamped` (`geometry_msgs/TwistStamped`, body-frame `linear.x`, `linear.y`, `angular.z`) |
| Localization and mapping (BIEVR-LIO) | remove the `bievr_lio` node from `launch/explore.launch.py` | `/lidar/points`, `/imu/data` | `/bievr_lio/odom` (`nav_msgs/Odometry`, frame `odom`, child `imu`), `/bievr_lio/points/registered` (point cloud in frame `odom`) and TF `odom → imu` |
| Simulation only | do not run `explore.sh` | `/lidar/points`, `/imu/data`, `/ground_truth/odom`, `/clock` | `/cmd_vel` or `/cmd_vel_stamped` |

Main topics:

| Topic | Type | Notes |
|---|---|---|
| `/lidar/points` | PointCloud2 | 10 Hz, frame `lidar`; fields `x y z intensity t`, `t` = nanoseconds since the start of the sweep (for deskewing), `intensity` always 0 |
| `/imu/data` | Imu | 200 Hz, frame `imu` (= body) |
| `/ground_truth/odom` | Odometry | 50 Hz ground truth, frame `world` |
| `/state_estimation`, `/registered_scan` | Odometry, PointCloud2 | localization result and registered point cloud, frame `map`; input of the planning modules |
| `/terrain_map`, `/terrain_map_ext` | PointCloud2 | terrain analysis, `intensity` = height of the point above the ground; ~10 m near / ~40 m far |
| `/way_point` | PointStamped | goal from the exploration planner to the local planner |
| `/cmd_vel_stamped` | TwistStamped | velocity command; the sim clamps it to ±1 m/s forward / sideways and ±0.6 rad/s yaw |
| `/sim/reset` | String | `here` stands the robot up in place, `spawn` puts it back at the start |

## Project layout

```
├── setup.sh               one-shot installation
├── sim.sh / explore.sh    terminal 1: simulation; terminal 2: algorithms
├── stop_all.sh            stops the simulation and all algorithm nodes
├── run_trial.sh           one unattended trial
├── container.sh           the project container isaac_explore: shell, build, rm
├── docker/                project Dockerfile, Isaac Lab patch, environment of the algorithm processes, Python wheel list
├── tools/                 download checkpoints and scenes (fetch_assets.py, add_scenes.sh), scene checks (check_scenes.sh,
│                          scene_spawn.py); resumable download of the Isaac Sim image
├── config/                per-robot parameters (robots.yaml), lidar models (lidars.yaml), scene start poses
│                          (scenes.yaml), BIEVR-LIO, TARE and elevation map configuration
├── launch/                launch file of the whole stack (explore.launch.py), CMU module launch files (cmu/)
├── sim/                   simulator go2_sim.py, scene conversion, ground-truth localization, evaluation / recording / diagnostic tools
├── ros2_ws/               third-party sources and pinned versions (src.repos), fetch script, patches
├── terrain_gen/           procedural terrain generation
├── ros1_ws/           GBPlanner2 in a ROS 1 container (image, bridge, adapter, parameters), comparison and test tools
├── assets/  third_party/  (downloaded by setup.sh / tools/add_scenes.sh) checkpoints, worlds and models with their
│                          attribution list (assets/ATTRIBUTION.md); Isaac Lab sources
└── logs/                  (written at run time) exploration metrics, trajectories, trial records
```

## Troubleshooting

| Problem | What to do |
|---|---|
| Downloads are slow or break off during installation | Run `./setup.sh` again; it carries on. In mainland China add `--cn` and see the proxy notes above |
| Compiling C++ during installation runs out of memory / freezes the desktop | 4 files are compiled at a time by default (~1.5 GB each). With less RAM: `ISAAC_EXPLORE_JOBS=2 ./setup.sh` |
| Step 6 (scene conversion) fails with exit code 137 | Killed for lack of memory: close other programs and run `./setup.sh` again; the render pipelines compiled so far stay cached |
| The simulation window or RViz does not open | Run `./sim.sh` again: it re-authorizes the container on the display (lost after an X restart or a new login). On a machine without a display: `./sim.sh --headless` and `./explore.sh rviz:=false` |
| `explore.sh` warns "no simulation running" | Start `./sim.sh` in terminal 1 first and wait for `[go2_sim] running` |
| Localization goes badly wrong, the trajectory jumps | Are two simulations running? Their topics get mixed up. `./stop_all.sh` and start again |
| The robot falls or gets stuck | See [Common tasks](#common-tasks). The walking policy is the official pretrained one, not trained for rubble and side slopes, and falls now and then. Tilted more than 60°, or more than 35° for 5 s (wedged among rocks), counts as fallen: with `--auto_reset 10` the sim stands it up after 10 s. Commanded to move but not getting 0.2 m or 15° in 4 s, it backs up and turns |
| Not enough memory, sluggish desktop | `./sim.sh --headless`, `./explore.sh elevation:=false` |
| Uninstall | `docker rm -f isaac_explore && docker rmi isaac_explore isaac-lab-base nvcr.io/nvidia/isaac-sim:4.5.0`, `docker volume ls -q \| grep isaac_explore_ \| xargs docker volume rm`, then delete the project directory |

## Known limitations

These are limits of the current algorithms, and also what makes the platform useful for research:

- **TARE is not good at winding, branching caves.** It aims to "see every visible surface well", so branches behind bends and narrow gaps tend to be given up, and in `simple_cave_01` it declares exploration complete early. At narrow places such as tunnel entrances it sometimes puts the waypoint where the robot cannot get to, and the robot turns back and forth nearby. Exploration strategies better suited to such places (e.g. topology-based ones) can be plugged in with `exploration:=false`.
- **The walking policy is the official, general-purpose pretrained one**, not trained for rubble and side slopes; it falls now and then while exploring (see [Common tasks](#common-tasks)), and its top speed is 1.0 m/s. If it does not stand still after spawning, `sim.sh` puts it back and prints `[go2_sim] running` only once it stands.
- **Slopes steeper than about 20° are a hard limit of the current stack.** The CMU terrain analysis takes the lowest point within ~0.5 m as the ground, so on a slope the slope itself rises above the 0.2 m obstacle threshold: on a smooth slope 9% of the points count as obstacles at 25° and 34% at 30° (none up to 20°), more on rough cave floors. The walking policy was trained on slopes up to ~22° (Isaac Lab `slope_range` 0.4); on test ramps it climbs and descends 15° reliably, gets stuck at the crest or the edge of 20–25° ramps and falls on 30–35°. Downhill the terrain analysis never sees an obstacle, so the robot may walk down a slope it cannot climb back up.
- **The local planner only drives forward.** A way point beside or behind the robot is aimed at as if it were 90° to the side, so instead of turning on the spot the robot may walk a wide circle before it turns round.
- **BIEVR-LIO sometimes ends up with a constant offset of a few metres after initialization**, and tracks steadily from then on. Planning happens in its own frame, so exploration is not affected; keep it in mind when comparing localization accuracy.
- **The simulation runs at about 0.4× real time.** All nodes use simulation time, so results are unaffected; it just looks slow.
- **The lidar is simulated with ray casting**: no intensity, static scenes only.

## Versions

| Version | Contents |
|---|---|
| v2.1.0 | lidar models selectable with `./sim.sh --lidar`: Ouster OS1-32 (default), OS1-64, OS1-128, OS0-128, Velodyne VLP-16, HDL-32E, Hesai PandarXT-32 (`config/lidars.yaml`); GBPlanner2's sensor model follows the vertical field of view; GBPlanner2 screenshots |
| v2.0.0 | GBPlanner2 as an optional exploration planner (ROS 1 container + `ros1_bridge`), with tools to compare it with TARE; 33 more scenes (`tools/add_scenes.sh`, `./sim.sh --scene NAME / --list`, start poses in `config/scenes.yaml`, floor slabs of SubT worlds visible to the height scan and the lidar); stuck and fall detection in the simulator that also catch a robot wedged among rocks; trajectory and local path in RViz; measured slope limits; scene attribution list |
| v1.0.0 | first release: Isaac Sim + Isaac Lab, ANYmal D / Go2, BIEVR-LIO, elevation map, CMU terrain analysis and local planner, TARE exploration in the SubT cave `simple_cave_01`. On a machine where its container was never created, `setup.sh` stops at step 6 without a message: fixed in v2.0.0 |

## License

The code, configuration and documentation in this repository are licensed under [BSD-3-Clause](LICENSE).

This repository contains only this project's own content. Third-party sources, models and checkpoints are downloaded from their original locations by `setup.sh` and keep their own licenses:

| Component | Used for | License |
|---|---|---|
| [Isaac Sim 4.5](https://developer.nvidia.com/isaac/sim) (NVIDIA) | physics, rendering | NVIDIA Isaac Sim license (accepted by starting the container, `ACCEPT_EULA=Y`) |
| [Isaac Lab v2.1.1](https://github.com/isaac-sim/IsaacLab) | robot environments, walking policies (official pretrained checkpoints) | BSD-3-Clause |
| [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO) (ETH Zurich ASL) | lidar-inertial localization and mapping | BSD-3-Clause |
| [elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy) (ETH Zurich RSL), [ros2_numpy](https://github.com/Box-Robotics/ros2_numpy) | elevation map | MIT |
| [TARE](https://github.com/caochao39/tare_planner), [autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment) (CMU) | exploration planner; terrain analysis, local planner | BSD |
| [terrain-generator](https://github.com/leggedrobotics/terrain-generator) (ETH Zurich RSL) | procedural terrain | MIT |
| [DARPA SubT worlds](https://github.com/osrf/subt) (Open Source Robotics Foundation, DARPA Subterranean Challenge virtual competition) | the cave, tunnel, NIOSH mine, finals and urban scenes: world layouts and tile graphs | Apache-2.0 |
| [Gazebo Fuel models](https://app.gazebosim.org/OpenRobotics) (Open Robotics) | the tiles and props those worlds are built from | CC BY 4.0 (most) or CC0, per model: `assets/ATTRIBUTION.md` |
| [gbplanner_ros](https://github.com/ntnu-arl/gbplanner_ros) (gbplanner2), [pci_general](https://github.com/ntnu-arl/pci_general), [voxblox](https://github.com/ntnu-arl/voxblox) (NTNU Autonomous Robots Lab) | GBPlanner2 (optional) | BSD-3-Clause; pci_general BSD-2-Clause |
| [mav_comm](https://github.com/ethz-asl/mav_comm), [minkindr](https://github.com/ethz-asl/minkindr), [catkin_simple](https://github.com/catkin/catkin_simple) and the `*_catkin` wrappers (ETH Zurich ASL) | GBPlanner2's build dependencies | Apache-2.0 / BSD |
| [adaptive_obb_ros](https://github.com/ntnu-arl/adaptive_obb_ros) (NTNU) | a GBPlanner2 dependency | **no license stated by its authors**: fetched from its repository when you build GBPlanner2, not redistributed here |
| [ros:noetic-ros-base](https://hub.docker.com/_/ros), [ros:foxy-ros1-bridge](https://hub.docker.com/_/ros) (Open Robotics) | base of the GBPlanner2 image; ROS 1 ↔ ROS 2 bridge | Apache-2.0 / BSD (ROS packages) |
| [gbplanner_ros](https://github.com/ntnu-arl/gbplanner_ros) sample worlds (NTNU Autonomous Robots Lab) | the `gbp_*` scenes: mine scans `edgar_mine`, `pittsburgh_mine`, `virginia_mine` (models by Tung Dang), the layouts of `darpa_cave_01/02` and `urban_cave_transition` (tiles from Fuel) | BSD-3-Clause |

**Scene assets**: `tools/fetch_assets.py` downloads the worlds and models from the sources above (they are not in this repository) and writes `assets/ATTRIBUTION.md`, the list of every downloaded model with its owner, version and license. `sim/sdf_to_usd.py` converts them for Isaac Sim (format conversion to USD, all collision geometry merged into one mesh, artifact models left out). If you publish results, images or videos made with these scenes, credit them as their licenses ask, e.g. "DARPA Subterranean Challenge worlds and models by Open Robotics, CC BY 4.0".

**Files in this repository adapted from third-party projects** (they keep the original license; each names its source at the top):

| File | Source | License |
|---|---|---|
| `launch/cmu/*.launch` | launch files of autonomous_exploration_development_environment, parameters changed | BSD |
| `config/tare_go2.yaml` | tare_planner's `config/tunnel.yaml`, parameters changed | BSD |
| `config/bievr_params.yaml` | BIEVR-LIO's `config/params.yaml`, two parameters changed | BSD-3-Clause |
| `ros2_ws/patches/*.patch` | small changes to the sources of tare_planner and autonomous_exploration_development_environment | BSD |
| `docker/isaaclab.patch` | changes to Isaac Lab's Docker configuration | BSD-3-Clause |
| `terrain_gen/patches/*.patch` | changes to terrain-generator | MIT |
| `ros1_ws/config/*.yaml`, `launch/gbplanner_isaac.launch` | GBPlanner2's SMB sample configuration and launch file, parameters changed | BSD-3-Clause |
| `ros1_ws/patches/gbplanner_ros.patch` | log level and two crash guards in GBPlanner2's source | BSD-3-Clause |

All other code, scripts and documentation were written for this project and use the components above only through their APIs.

**Citation**: this project integrates the open-source work above into a ready-to-run simulation environment. If it helps your research, please cite the original papers of the components (see each project's page for how to cite it).

**Note**: the image that `setup.sh` builds locally contains Isaac Sim, which NVIDIA's license does not allow to be redistributed; do not push it to a public registry.
