# Terra Incognita

[English](README.md) | **中文**

**四足机器人在未知地下环境中自主探索的仿真平台。** 基于 Isaac Sim 和 ROS 2，一个脚本装好全部环境，两个终端就能跑起"定位建图 + 自主探索 + 运动控制"的完整闭环。适合在研究的前期快速搭好仿真场景，再把其中某个模块换成自己的算法。

<p align="center">
  <img src="media/isaac_sim.png" width="49%" alt="Isaac Sim：ANYmal D 站在 SubT 洞穴的出发区（半透明显示）">
  <img src="media/rviz.png" width="49%" alt="RViz：定位建图与 TARE 探索规划">
</p>

## 功能

- **场景**：DARPA SubT 的洞穴 `simple_cave_01`（`setup.sh` 自动安装）；另有 33 个场景一条命令即可添加：其他 SubT 洞穴、矿道、NIOSH 真实矿井复刻、决赛和城市场地、真实矿洞扫描，以及程序化生成的受限地形（见[更多场景](#更多场景)）。
- **机器人**：ANYmal D（默认）和 Unitree Go2，使用 Isaac Lab 官方预训练的复杂地形行走策略，接收速度指令。
- **传感器**：模拟 Ouster OS1-32 激光雷达（10 Hz，带逐点时间戳和真实的运动畸变）、200 Hz IMU、真值位姿。
- **算法**：
  - [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO)：激光惯性定位建图；
  - [elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy)：GPU 高程图；
  - CMU 的[地形分析、局部规划、路径跟踪](https://github.com/HongbiaoZ/autonomous_exploration_development_environment)；
  - [TARE](https://github.com/caochao39/tare_planner)：自主探索规划；
  - 可选：用 [GBPlanner2](https://github.com/ntnu-arl/gbplanner_ros/tree/gbplanner2) 代替 TARE，作为对比探索策略的基线（见 [GBPlanner2](#gbplanner2可选实验性)）。
- **一个 Docker 镜像**包含全部环境（Isaac Sim 4.5、Isaac Lab 2.1.1、ROS 2 Humble、各算法的依赖），第三方源码都锁定了版本，可以复现。
- **方便替换**：定位、探索规划等任何一层都可以换成你自己的节点（见[接入自己的算法](#接入自己的算法)）。

```
┌──────────────────────── Isaac Sim + Isaac Lab（sim.sh）────────────────────────┐
│  物理仿真 ─ 行走策略（速度指令 → 关节）─ 模拟激光雷达 / IMU / 真值             │
└──────┬───────────────────────────────────────────────────────────▲─────────────┘
       │ /lidar/points  /imu/data                                  │ /cmd_vel_stamped
┌──────▼───────────────────────── ROS 2 算法（explore.sh）─────────┴─────────────┐
│  BIEVR-LIO 定位建图 ─► 地形分析 ─► TARE 探索规划 ─► 局部规划 ─► 路径跟踪       │
│                    └─► 高程图                                                  │
└────────────────────────────────────────────────────────────────────────────────┘
```

## 系统要求

| 项目 | 要求 | 测试环境 |
|---|---|---|
| 系统 | Linux x86_64（Ubuntu 20.04 / 22.04） | Ubuntu 20.04 |
| 显卡 | NVIDIA RTX，显存 ≥ 8 GB | RTX 4080 SUPER 16 GB |
| 驱动 | ≥ 535.129（Isaac Sim 4.5 的要求） | 550 |
| 内存 | 32 GB（第一次启动 Isaac Sim 时编译渲染管线，峰值约 21 GB，之后约 5 GB） | 32 GB |
| 磁盘 | Docker 所在分区约 70 GB 可用空间 | |
| 网络 | 首次安装约下载 30 GB | |

激光雷达是用射线投射模拟的，不需要 Isaac Sim 的 RTX 雷达（那需要 570 以上的驱动）。

## 安装

### 第 0 步：准备主机环境

需要 NVIDIA 驱动、Docker、NVIDIA Container Toolkit。已经装好的可以跳过，`setup.sh` 第一步也会检查。

```bash
nvidia-smi                                   # 能看到显卡和驱动版本（>= 535）即可

# Docker（官方安装脚本），并允许当前用户直接使用 docker（之后要重新登录）
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER

# NVIDIA Container Toolkit（详见 https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html）
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker

docker run --rm --gpus all ubuntu nvidia-smi  # 验证：容器里能看到显卡
```

### 第 1 步：下载项目

```bash
git clone https://github.com/ygd551/terra-incognita.git
cd terra-incognita
```

### 第 2 步：一键安装

```bash
./setup.sh          # 在中国大陆建议用 ./setup.sh --cn（构建镜像时使用清华的 apt 镜像源）
```

`setup.sh` 依次完成下面 7 步。**中途失败（比如断网）直接重新运行即可**，已经完成的步骤会跳过：

| 步骤 | 内容 | 首次耗时（参考） |
|---|---|---|
| 1 | 检查驱动、Docker、NVIDIA Container Toolkit、磁盘空间 | – |
| 2 | 按固定版本拉取第三方 ROS 2 源码（BIEVR-LIO、elevation_mapping_cupy、TARE、CMU 环境），打上本项目的补丁 | 几分钟 |
| 3 | 克隆 Isaac Lab v2.1.1 并打补丁，构建 Isaac Sim + Isaac Lab 基础镜像 | 1–2 小时 |
| 4 | 构建项目镜像 `isaac_explore`：ROS 2 Humble 和各算法的依赖 | 约 40 分钟 |
| 5 | 下载行走策略权重（Isaac Lab 官方发布）和 SubT 洞穴场景（约 100 MB） | 几分钟 |
| 6 | 把场景从 Gazebo 的 SDF 格式转换成 Isaac Sim 的 USD 格式 | 几分钟 |
| 7 | 编译 ROS 2 工作区 | 约 2 分钟 |

最后打印 `Installation complete` 就说明全部就绪。

第 6 步是第一次启动 Isaac Sim，要编译渲染管线，内存峰值约 21 GB，只发生这一次。内存是 32 GB 的话，安装时建议关掉浏览器等占内存的程序。所有进程都运行在一个有内存上限的容器里（默认是总内存减 9 GB），即使超出，也只会终止容器里的进程，不会让整机卡死。

- 机器上已经有 Isaac Lab v2.1.1 的源码：`ISAACLAB_DIR=/path/to/IsaacLab ./setup.sh`，不会再克隆一份。
- `docker pull` Isaac Sim 镜像（15 GB）反复失败时，脚本会自动改用支持断点续传的下载方式（`tools/fetch_image.sh`）。
- Docker 拉镜像需要代理时，配置 Docker daemon 的代理（`/etc/systemd/system/docker.service.d/http-proxy.conf`）；构建时用的代理写在 `~/.docker/config.json` 的 `proxies` 里。

**更新项目**：`git pull && ./setup.sh`，会按需要重建镜像、重新编译，没有变化的步骤几秒钟就结束。

## 运行

打开两个终端，都进入项目目录。

**终端 1：启动仿真**

```bash
./sim.sh
```

第一次启动需要几分钟（编译着色器、下载机器人模型）。弹出 Isaac Sim 窗口，ANYmal D 出现在洞穴入口并站稳，终端打印 `[go2_sim] running` 后，再进行下一步。

**终端 2：启动算法**

```bash
./explore.sh
```

几秒后会弹出 RViz 窗口。终端里出现 `Exploration Started` 后，机器人开始自主探索：自己决定去哪里，走进洞穴，绕开岩石。探索完成后会打印 `Exploration completed, returning home`，然后返回出发点。

**结束**：在任意终端运行 `./stop_all.sh`。

### 运行时能看到什么

**Isaac Sim 窗口**：洞穴是封闭的，从外面看不到里面的机器人。可以用 `./sim.sh --see_through 0.25` 启动：洞穴画成半透明的，相机从机器人上方跟随它（上图）。洞穴深处的岩层较多，画面会变暗，可以把数值调小（如 0.1，越小越透明）。这个选项只影响显示，不影响物理和传感器。探索过程在 RViz 里看得最清楚。

**RViz 窗口**中的主要符号：

| 符号 | 含义 |
|---|---|
| 蓝色的点云 | 已探索的区域（配准后的激光点累积而成） |
| 红、绿短线 | 机器人的位姿：红色是机头方向，绿色是左侧 |
| 大的绿色方框 | TARE 的局部规划范围（30 m × 30 m），只在框内挑选视点 |
| 小的绿色方块 | 还没探完、排了顺序的子空间，颜色越实越先去。全部消失后探索结束 |
| 浅青色的线 / 深蓝色的线 | 全局路线 / 局部路线 |
| 橙色的点 | 选中的视点：机器人要过去"看一眼"的位置 |
| 红色的点 | 还没看清的表面和前沿（已知与未知空间的交界） |
| 紫色的点 | 当前航点：前方约 5 m，发给局部规划器的目标 |
| 机器人前方的亮绿色线 | 局部规划器选出的路径（约 4 m，避开障碍） |
| 黄色的点 | 走过的轨迹 |

**探索指标**：每次运行都会在 `logs/` 里生成 `metrics_<时间>.txt`，每行 4 列依次是 **已探索体积（m³）、行走距离（m）、算法耗时、经过的时间（s）**；同目录的 `trajectory_<时间>.txt` 是机器人的轨迹。

## 使用说明

### 启动选项

| 命令 | 选项 | 作用 |
|---|---|---|
| `./sim.sh` | `--robot go2` | 换成 Unitree Go2（默认 ANYmal D）。`explore.sh` 会自动使用同一种机器人 |
| | `--headless` | 不开仿真窗口，省显存和内存，只看 RViz |
| | `--scene 场景名` | 换场景（`./sim.sh --list` 列出可用场景，见[更多场景](#更多场景)），出生位姿取自 `config/scenes.yaml` |
| | `--spawn X Y [Z]` `--yaw 角度` | 换出生位置和朝向；不给 Z 时自动放到 (X, Y) 处的地面上 |
| | `--see_through 0.25` | 场景半透明显示，相机跟随机器人 |
| | `--auto_reset 10` | 摔倒 10 s 后原地自动扶正（配合 `lio:=gt` 做无人值守实验） |
| | `--help` | 全部选项 |
| `./explore.sh` | `lio:=gt` | 用仿真真值代替 BIEVR-LIO 定位，单独测试规划 |
| | `speed:=0.5` | 最高速度，默认 1.0 m/s（行走策略训练时的上限，不要再提高） |
| | `exploration:=false` | 不启动 TARE，其余照常运行，由你自己发布 `/way_point` |
| | `exploration:=gbplanner` | 用 GBPlanner2 代替 TARE 探索（需先安装，见 [GBPlanner2](#gbplanner2可选实验性)） |
| | `planner:=false` | 只跑定位建图（不启动探索和局部规划） |
| | `rviz:=false`、`elevation:=false` | 不开 RViz、不跑高程图 |

### 常用操作

```bash
# 机器人摔倒时（仿真终端打印 "robot has fallen"）：here = 原地扶正，spawn = 送回出发点
docker exec isaac_explore bash -c 'source /opt/ros/humble/setup.bash && ros2 topic pub --once /sim/reset std_msgs/msg/String "{data: here}"'
#   扶正后位姿会跳变，BIEVR 无法继续跟踪；用 BIEVR 时更简单的办法是 ./stop_all.sh 后重新开始

# 进入容器，运行 ros2 命令或项目里的工具脚本
./container.sh
source /isaac_explore/docker/algo_env.sh
ros2 topic list
python3 /isaac_explore/sim/watch_explore.py 600       # 实时打印位置、行走距离、航点
python3 /isaac_explore/sim/eval_lio.py                # 按固定路线行走，评估 BIEVR-LIO 相对真值的误差
python3 /isaac_explore/sim/drive_test.py 0.5 0 0 10   # 手动发速度指令：vx vy wz 持续秒数（先停掉 explore.sh）

# 无人值守跑一次实验（无界面仿真 + 全部算法 + 记录），结果在 logs/trial_<名字>/
LAUNCH_ARGS="lio:=gt" ./run_trial.sh 名字 [最长仿真秒数] [sim.sh 的选项，例如 --scene simple_tunnel_01]
```

### 修改参数

改完后重启对应的终端即可。配置和 launch 文件都是直接从项目目录读取的，不需要重新编译。

| 想改什么 | 改哪里 | 生效方式 |
|---|---|---|
| 最高速度 | `./explore.sh speed:=0.5` | 重启终端 2 |
| 机器人、场景、出生位姿 | `./sim.sh --robot / --scene / --spawn / --yaw`；各场景的默认出生位姿在 `config/scenes.yaml` | 重启两个终端 |
| 雷达安装位置、车体尺寸、障碍高度阈值、视点高度、出生高度 | `config/robots.yaml` | 重启两个终端。雷达位置还要同步修改 `config/bievr_sim_<robot>.yaml` 的 `calibration` |
| BIEVR-LIO | `config/bievr_params.yaml`、`config/bievr_sim_<robot>.yaml` | 重启终端 2 |
| TARE 探索规划 | `config/tare_go2.yaml`（两种机器人共用） | 重启终端 2 |
| 地形分析、局部规划、路径跟踪（转向速度、加速度等） | `launch/cmu/*.launch` | 重启终端 2 |
| 高程图 | `config/elevation_sim.yaml` | 重启终端 2 |
| 仿真端的速度上限、转弯降速、卡住脱困、摔倒判定 | `sim/go2_sim.py` 开头的常量 | 重启终端 1 |
| 第三方包的源码 | `ros2_ws/src/<包>/` | `./container.sh build` 后重启终端 2。要长期保留的修改，做成补丁放进 `ros2_ws/patches/`，并登记到 `ros2_ws/fetch_src.sh` |
| 容器内存上限（默认：总内存减 9 GB，16–26 GB 之间） | `ISAAC_EXPLORE_MEM=24g ./container.sh rm`，之后正常启动 | 重建容器 |

### 更多场景

**其他 SubT 世界和示例矿洞。** `tools/add_scenes.sh 场景名...` 会下载世界文件和它用到的模型，并转换成 `sim/scenes/场景名.usd`；`./sim.sh --list` 列出已有的场景，`./sim.sh --scene 场景名` 按默认出生位姿（`config/scenes.yaml`）启动：

```bash
tools/add_scenes.sh simple_tunnel_01 niosh_sr_config_a   # 场景名：osrf/subt 的世界名，或已放在 assets/subt/worlds/ 下的 .sdf
./sim.sh --list
./sim.sh --scene simple_tunnel_01
tools/check_scenes.sh simple_tunnel_01                  # 可选：检查机器狗能否在出生点站稳
```

| 类型 | 场景（[osrf/subt](https://github.com/osrf/subt/tree/master/subt_ign/worlds) 的世界都带分块拓扑图，可以统计覆盖率） |
|---|---|
| 天然洞穴 | `simple_cave_01..03`、`cave_qual`、`cave_circuit_practice_01..03` |
| 矿道 | `simple_tunnel_01..03`、`tunnel_qual_ign`、`tunnel_circuit_practice_01..03` |
| 真实矿井复刻（NIOSH） | `niosh_sr_config_a/b`、`niosh_ex_config_a/b` |
| 综合决赛场地 | `finals_practice_01..03`、`finals_qual` |
| 城市地下 | `simple_urban_01..03`、`urban_circuit_practice_01..03` |
| [gbplanner_ros](https://github.com/ntnu-arl/gbplanner_ros) 示例（`ros1_ws/add_gbp_scenes.sh`） | `gbp_darpa_cave_01/02`（与 `simple_cave_01/02` 布局相同，分块较旧）、`gbp_edgar_mine`、`gbp_pittsburgh_mine`、`gbp_virginia_mine`（真实矿洞扫描）、`gbp_urban_cave_transition` |

全部 33 个额外场景约需下载 2 GB、占用磁盘约 4 GB，每个场景转换约需 15–60 s。这些场景的出处和许可证见[许可证](#许可证)一节。SubT 世界的出生点在比赛的集结区，即赛道入口前 8 m。新增的世界需要在 `config/scenes.yaml` 里加一行（`tools/scene_spawn.py 场景名 X Y` 可以帮忙找一块空地）。SubT 场景里的人造物（背包、假人、手机等）默认不放进场景，见 `sim/sdf_to_usd.py --skip`。

**程序化生成的受限地形**（[leggedrobotics/terrain-generator](https://github.com/leggedrobotics/terrain-generator)）：把墙、台阶、平台、悬垂结构等部件拼成 40 m × 40 m 的地图，适合模拟狭窄、多层的坍塌空间。需要主机上有一个装了 torch 和 open3d 的 Python：

```bash
PY=/path/to/python terrain_gen/setup.sh        # 第一次：拉取固定版本的源码、打补丁、建虚拟环境
terrain_gen/.venv/bin/python terrain_gen/generate.py --cfg indoor --enable_sdf --mesh_dir $PWD/terrain_gen/out/indoor
#   --cfg: indoor | overhanging | overhanging_floor；一次生成 10 张地图，约 5 分钟
rm -f terrain_gen/out/*/*/{sdf,mesh_terrain_sdf,mesh_terrain_distance}.npy   # 仿真用不到的距离场，每张图约 470 MB
docker exec isaac_explore bash -c 'cd /workspace/isaac_explore && /isaac-sim/python.sh sim/mesh_to_usd.py \
  terrain_gen/out/indoor/mesh_5/mesh.obj sim/scenes/tg_indoor_5.usd --spawn terrain_gen/out/indoor/mesh_5/spawnable_locations.npy'
#   最后一行打印建议的 --spawn X Y Z（按 Go2 的站立高度；ANYmal D 的 Z 再加约 0.23 m）
./sim.sh --scene tg_indoor_5 --spawn X Y Z           # 也可以把出生位姿写进 config/scenes.yaml
```

## GBPlanner2（可选，实验性）

[GBPlanner2](https://github.com/ntnu-arl/gbplanner_ros/tree/gbplanner2) 是挪威科技大学 Autonomous Robots Lab 的基于图的探索规划器，DARPA 地下挑战赛冠军队 CERBERUS 使用过。它可以代替 TARE 探索，作为对比探索策略的基线。它只有 ROS 1 Noetic 版本，所以运行在单独的容器里（镜像 `gbplanner_ros1`），通过 `ros1_bridge`（镜像 `ros:foxy-ros1-bridge`）和 ROS 2 算法栈通信。经过桥接的只有里程计、配准后的点云和 `/clock`；适配节点（`ros1_ws/scripts/isaac_adapter.py`）沿 GBPlanner 的路径向 CMU 局部规划器发送前方约 5 m 的航点，和 TARE 的做法一样。接入尽量保持原样：GBPlanner2 固定在一个提交，只加了两处防止上游崩溃的保护；参数取自它的地面机器人示例（SMB），按 ANYmal D 和 OS1-32 雷达做了调整（`ros1_ws/config/`）。

**安装**（只需一次；约 30 分钟，镜像约 7.5 GB）：

```bash
ros1_ws/fetch_src.sh            # 按固定版本拉取 GBPlanner2 及其依赖，并打上我们的补丁
docker build --network host -t gbplanner_ros1 -f ros1_ws/docker/Dockerfile ros1_ws
#   国内可加：--build-arg UBUNTU_MIRROR=mirrors.tuna.tsinghua.edu.cn
#            --build-arg ROS_APT_MIRROR=http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu
docker pull ros:foxy-ros1-bridge
ros1_ws/ros1.sh build           # 在容器里用 catkin 编译，约 2 分钟
```

**运行**：终端 1 照常运行 `./sim.sh`；终端 2 运行 `./explore.sh exploration:=gbplanner`（按 Ctrl-C 会一并停掉 GBPlanner）。机器狗会先直行 4 m：雷达看不到机器狗周围约 2 m 内的地面，而 GBPlanner 只在看到过的地面上规划。另一个 RViz 窗口显示 GBPlanner 自己的视图：局部图、带前沿（红色）的全局图、选中的路径、走过的历史位置。日志：`docker exec gbplanner_ros1 tail -f /tmp/gbplanner_launch.log /tmp/isaac_adapter.log`。

**对比和测试工具**（在 `ros1_ws/` 下）：

| 工具 | 作用 |
|---|---|
| `compare.sh [次数] [秒数]` | 在相同条件下，TARE 和 GBPlanner2 各做若干次无人值守的无头试验，最后输出每次的覆盖情况（`logs/compare_<时间>.txt`） |
| `run_trial.sh 名字 [秒数] [sim.sh 的选项]` | 一次无人值守的 GBPlanner2 试验，用法同 `./run_trial.sh` |
| `scripts/tile_coverage.py logs/trial_X/run.csv [metrics.txt]` | 随仿真时间统计进入的 SubT 分块数、行走距离和探索体积（任意 SubT 世界，`--world 场景名`） |
| `ramp_test.sh`、`scripts/cmu_slope_test.py` | 行走策略能上下多陡的坡；CMU 地形分析从多大坡度开始把坡面判为障碍 |
| `add_gbp_scenes.sh` | 把 gbplanner_ros 的示例世界加为场景（`gbp_*`，见[更多场景](#更多场景)） |

**已知表现**：GBPlanner 用它自己的 voxblox 地图规划，CMU 局部规划器按地形分析的结果行走。两者判断不一致时（主要是超过约 20° 的坡，见[已知限制](#已知限制)），机器狗会来回折返，直到适配节点在 20 s 无进展后放弃这条路径、让 GBPlanner 重新规划。GBPlanner 偏好未知体积大的方向，所以往往先探开阔的空腔，再进狭窄的巷道。

## 接入自己的算法

所有模块都通过 ROS 2 话题连接（`use_sim_time`，时钟是仿真发布的 `/clock`）。想替换哪一层，就关掉它，让你的节点订阅上游的话题、发布下游的话题：

| 想替换的模块 | 怎么关掉原模块 | 你的节点订阅 | 你的节点发布 |
|---|---|---|---|
| 探索规划（TARE） | `./explore.sh exploration:=false` | `/state_estimation`、`/registered_scan`、`/terrain_map`、`/terrain_map_ext` | `/way_point`（`geometry_msgs/PointStamped`，frame `map`，前方几米处的目标点） |
| 局部规划和控制 | `./explore.sh planner:=false` | 同上，加上自己的目标 | `/cmd_vel_stamped`（`geometry_msgs/TwistStamped`，机身坐标系的 `linear.x`、`linear.y`、`angular.z`） |
| 定位建图（BIEVR-LIO） | 在 `launch/explore.launch.py` 里去掉 `bievr_lio` 节点 | `/lidar/points`、`/imu/data` | `/bievr_lio/odom`（`nav_msgs/Odometry`，frame `odom`，child `imu`）、`/bievr_lio/points/registered`（frame `odom` 的点云）和 TF `odom → imu` |
| 只用仿真 | 不运行 `explore.sh` | `/lidar/points`、`/imu/data`、`/ground_truth/odom`、`/clock` | `/cmd_vel` 或 `/cmd_vel_stamped` |

主要话题：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/lidar/points` | PointCloud2 | 10 Hz，frame `lidar`；字段 `x y z intensity t`，`t` 是相对扫描起点的纳秒数（可以去畸变），`intensity` 恒为 0 |
| `/imu/data` | Imu | 200 Hz，frame `imu`（= 机身） |
| `/ground_truth/odom` | Odometry | 50 Hz 真值，frame `world` |
| `/state_estimation`、`/registered_scan` | Odometry、PointCloud2 | 定位结果和配准后的点云，frame `map`，规划模块的输入 |
| `/terrain_map`、`/terrain_map_ext` | PointCloud2 | 地形分析结果，`intensity` = 点的离地高度；近处约 10 m / 远处约 40 m |
| `/way_point` | PointStamped | 探索规划给局部规划的目标点 |
| `/cmd_vel_stamped` | TwistStamped | 速度指令，仿真限幅到前后、左右 ±1 m/s，转向 ±0.6 rad/s |
| `/sim/reset` | String | `here` 原地扶正，`spawn` 送回出发点 |

## 项目结构

```
├── setup.sh               一键安装
├── sim.sh / explore.sh    终端 1：仿真；终端 2：算法
├── stop_all.sh            停掉仿真和所有算法节点
├── run_trial.sh           无人值守跑一次实验
├── container.sh           项目容器 isaac_explore：进入、编译（build）、删除（rm）
├── docker/                项目镜像的 Dockerfile、Isaac Lab 补丁、算法进程的环境脚本、Python 包清单
├── tools/                 下载权重和场景（fetch_assets.py、add_scenes.sh）、检查场景（check_scenes.sh、scene_spawn.py）、
│                          断点续传下载 Isaac Sim 镜像
├── config/                每种机器人的参数（robots.yaml）、各场景的出生位姿（scenes.yaml），BIEVR-LIO、TARE、高程图的配置
├── launch/                整个算法栈的 launch（explore.launch.py），CMU 模块的 launch（cmu/）
├── sim/                   仿真主程序 go2_sim.py、场景转换、真值定位，以及评估、记录、诊断工具
├── ros2_ws/               第三方源码的地址和固定版本（src.repos）、拉取脚本、补丁
├── terrain_gen/           程序化地形生成
├── ros1_ws/           在 ROS 1 容器中运行的 GBPlanner2（镜像、桥接、适配节点、参数），以及对比和测试工具
├── assets/  third_party/  （由 setup.sh / tools/add_scenes.sh 下载）策略权重、世界文件和模型及其署名清单
│                          （assets/ATTRIBUTION.md）；Isaac Lab 源码
└── logs/                  （运行时生成）探索指标、轨迹、实验记录
```

## 常见问题

| 问题 | 处理 |
|---|---|
| 安装时下载很慢或中断 | 重新运行 `./setup.sh`，会接着做。在中国大陆加 `--cn`，并参考上面的代理设置 |
| 安装时编译 C++ 导致内存不足、系统卡顿 | 默认同时编译 4 个文件（每个约 1.5 GB 内存）。内存较小时用 `ISAAC_EXPLORE_JOBS=2 ./setup.sh` |
| 第 6 步场景转换失败，退出码 137 | 内存不足被终止：关掉其他程序后重新运行 `./setup.sh`，已经编译的渲染管线会保留在缓存里 |
| 仿真窗口或 RViz 打不开 | 重新运行 `./sim.sh`，它会重新授权容器访问显示（X 重启或重新登录后授权会丢失）。没有显示器的机器用 `./sim.sh --headless` 和 `./explore.sh rviz:=false` |
| `explore.sh` 提示 no simulation running | 先在终端 1 启动 `./sim.sh`，等它打印 `[go2_sim] running` |
| 定位明显出错、轨迹乱跳 | 是否同时开了两个仿真？两个仿真的话题会混在一起。`./stop_all.sh` 后重新开始 |
| 机器人摔倒或卡住 | 见[常用操作](#常用操作)。行走策略是官方预训练的，没有专门针对碎石和侧坡训练，偶尔会摔。倾斜超过 60°，或者超过 35° 持续 5 s（卡在石头之间），都算摔倒：加了 `--auto_reset 10` 时，仿真会在 10 s 后把它扶起来。让它走或转、它 4 s 内却移动不到 0.2 m 也转不到 15° 时，会自动后退并转向脱困 |
| 内存不够、系统卡顿 | `./sim.sh --headless`、`./explore.sh elevation:=false` |
| 卸载 | `docker rm -f isaac_explore && docker rmi isaac_explore isaac-lab-base nvcr.io/nvidia/isaac-sim:4.5.0`，`docker volume ls -q \| grep isaac_explore_ \| xargs docker volume rm`，然后删除项目目录 |

## 已知限制

这些是现有算法本身的局限，也是这个平台适合拿来做研究的地方：

- **TARE 不擅长曲折、多分支的矿洞。** 它的目标是"把看得到的表面都看清"，转弯和窄缝后面的分支容易被放弃，在 `simple_cave_01` 里会提前宣布探索完成。在矿道入口这类狭窄处，它有时会把航点放在走不到的位置，机器人会在附近来回打转。更适合这类场景的探索策略（例如基于拓扑的方法）可以通过 `exploration:=false` 接入。
- **行走策略是官方预训练的通用策略**，没有专门针对碎石和侧坡训练，探索途中偶尔会摔倒（见[常用操作](#常用操作)），速度上限 1.0 m/s。出生时如果没站稳，`sim.sh` 会自动重新放置，站稳后才打印 `[go2_sim] running`。
- **超过约 20° 的坡是当前整套系统的硬限制。** CMU 地形分析把约 0.5 m 范围内最低的点当作地面，坡一陡，坡面本身就会高出 0.2 m 的障碍阈值：在平整坡面上，25° 时有 9% 的点被当成障碍，30° 时 34%（20° 以内为 0），在起伏的洞穴地面上会更多。行走策略训练时的坡度最大约 22°（Isaac Lab 的 `slope_range` 为 0.4）；在测试斜坡上，15° 能稳定上下，20–25° 会在坡顶或坡口卡住，30–35° 会摔倒。下坡时地形分析看不到障碍，所以机器人可能走下一个它爬不回来的坡。
- **局部规划器只会向前走。** 航点在机器人侧面或身后时，会被当成正左或正右方来处理，机器人可能绕一个大圈才掉头，而不是原地转身。
- **BIEVR-LIO 初始化后偶尔会出现几米的固定偏移**，之后跟踪稳定。规划都在它自己的坐标系里进行，不影响探索；对比定位精度时要注意。
- **仿真大约是实时的 0.4 倍**，所有节点都用仿真时间，结果不受影响，只是看起来比较慢。
- **激光雷达是射线投射模拟的**：没有强度信息，只能扫到静态场景。

## 版本

| 版本 | 内容 |
|---|---|
| v2.0.0 | 新增可选的探索规划器 GBPlanner2（ROS 1 容器 + `ros1_bridge`），以及与 TARE 对比的工具；新增 33 个场景（`tools/add_scenes.sh`、`./sim.sh --scene 场景名 / --list`，出生位姿在 `config/scenes.yaml`，SubT 世界中的地板盒子对高度扫描和雷达可见）；仿真端的卡住和摔倒判定也能识别卡在石头间的情况；RViz 中显示轨迹和局部路径；实测的坡度上限；场景署名清单 |
| v1.0.0 | 首个版本：Isaac Sim + Isaac Lab，ANYmal D / Go2，BIEVR-LIO，高程图，CMU 地形分析和局部规划，SubT 洞穴 `simple_cave_01` 中的 TARE 探索。在从未创建过容器的机器上，`setup.sh` 会在第 6 步无提示地退出，v2.0.0 已修复 |

## 许可证

本仓库的代码、配置和文档采用 [BSD-3-Clause](LICENSE) 许可证。

本仓库只包含本项目自己的内容。第三方源码、模型和权重都由 `setup.sh` 从原始地址下载，遵循各自的许可证：

| 组件 | 用途 | 许可证 |
|---|---|---|
| [Isaac Sim 4.5](https://developer.nvidia.com/isaac/sim)（NVIDIA） | 物理仿真、渲染 | NVIDIA Isaac Sim 许可协议（启动容器即表示接受，`ACCEPT_EULA=Y`） |
| [Isaac Lab v2.1.1](https://github.com/isaac-sim/IsaacLab) | 机器人环境、行走策略（官方预训练权重） | BSD-3-Clause |
| [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO)（ETH Zurich ASL） | 激光惯性定位建图 | BSD-3-Clause |
| [elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy)（ETH Zurich RSL）、[ros2_numpy](https://github.com/Box-Robotics/ros2_numpy) | 高程图 | MIT |
| [TARE](https://github.com/caochao39/tare_planner)、[autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment)（CMU） | 探索规划；地形分析、局部规划 | BSD |
| [terrain-generator](https://github.com/leggedrobotics/terrain-generator)（ETH Zurich RSL） | 程序化地形 | MIT |
| [DARPA SubT 世界](https://github.com/osrf/subt)（Open Source Robotics Foundation，DARPA 地下挑战赛虚拟赛） | 洞穴、矿道、NIOSH 矿井、决赛场地和城市场景的布局与分块拓扑图 | Apache-2.0 |
| [Gazebo Fuel 模型](https://app.gazebosim.org/OpenRobotics)（Open Robotics） | 这些世界用到的分块和物品模型 | 多数为 CC BY 4.0，少数为 CC0，逐个模型见 `assets/ATTRIBUTION.md` |
| [gbplanner_ros](https://github.com/ntnu-arl/gbplanner_ros)（gbplanner2）、[pci_general](https://github.com/ntnu-arl/pci_general)、[voxblox](https://github.com/ntnu-arl/voxblox)（NTNU Autonomous Robots Lab） | GBPlanner2（可选） | BSD-3-Clause；pci_general 为 BSD-2-Clause |
| [mav_comm](https://github.com/ethz-asl/mav_comm)、[minkindr](https://github.com/ethz-asl/minkindr)、[catkin_simple](https://github.com/catkin/catkin_simple) 和各个 `*_catkin` 封装（ETH Zurich ASL） | GBPlanner2 的编译依赖 | Apache-2.0 / BSD |
| [adaptive_obb_ros](https://github.com/ntnu-arl/adaptive_obb_ros)（NTNU） | GBPlanner2 的一个依赖 | **作者没有声明许可证**：编译 GBPlanner2 时从其仓库拉取，本仓库不分发它的代码 |
| [ros:noetic-ros-base](https://hub.docker.com/_/ros)、[ros:foxy-ros1-bridge](https://hub.docker.com/_/ros)（Open Robotics） | GBPlanner2 镜像的基础；ROS 1 与 ROS 2 之间的桥接 | Apache-2.0 / BSD（ROS 软件包） |
| [gbplanner_ros](https://github.com/ntnu-arl/gbplanner_ros) 示例世界（NTNU Autonomous Robots Lab） | `gbp_*` 场景：矿洞扫描 `edgar_mine`、`pittsburgh_mine`、`virginia_mine`（模型作者 Tung Dang），以及 `darpa_cave_01/02`、`urban_cave_transition` 的布局（分块来自 Fuel） | BSD-3-Clause |

**场景资源**：`tools/fetch_assets.py` 从上面的原始出处下载世界文件和模型（它们不在本仓库中），并生成 `assets/ATTRIBUTION.md`，列出每个下载的模型及其发布者、版本和许可证。`sim/sdf_to_usd.py` 把它们转换成 Isaac Sim 可用的格式（转为 USD、所有碰撞几何合并成一个网格、不放入人造物模型）。如果发表用这些场景得到的结果、图片或视频，请按各自许可证的要求注明出处，例如“DARPA Subterranean Challenge worlds and models by Open Robotics, CC BY 4.0”。

**本仓库中改编自第三方的文件**（保留原许可证，文件开头注明了来源）：

| 文件 | 来源 | 许可证 |
|---|---|---|
| `launch/cmu/*.launch` | autonomous_exploration_development_environment 的 launch 文件，改了参数 | BSD |
| `config/tare_go2.yaml` | tare_planner 的 `config/tunnel.yaml`，改了参数 | BSD |
| `config/bievr_params.yaml` | BIEVR-LIO 的 `config/params.yaml`，改了两个参数 | BSD-3-Clause |
| `ros2_ws/patches/*.patch` | 对 tare_planner、autonomous_exploration_development_environment 源码的小改动 | BSD |
| `docker/isaaclab.patch` | 对 Isaac Lab Docker 配置的改动 | BSD-3-Clause |
| `terrain_gen/patches/*.patch` | 对 terrain-generator 的改动 | MIT |
| `ros1_ws/config/*.yaml`、`launch/gbplanner_isaac.launch` | GBPlanner2 的 SMB 示例配置和启动文件，改了参数 | BSD-3-Clause |
| `ros1_ws/patches/gbplanner_ros.patch` | GBPlanner2 源码中的日志级别和两处防崩溃保护 | BSD-3-Clause |

其余代码、脚本和文档是本项目自己写的，只通过 API 调用上面这些组件。

**引用**：本项目把上面这些开源工作整合成一个可以直接运行的仿真环境。如果它对你的研究有帮助，请引用各组件的原始论文（引用格式见各项目的主页）。

**注意**：`setup.sh` 在本地构建的镜像里包含 Isaac Sim，NVIDIA 的许可协议不允许再分发，请不要把它上传到公开的镜像仓库。
