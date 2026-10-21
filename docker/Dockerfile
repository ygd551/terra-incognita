# The one image of the project: Isaac Sim 4.5 + Isaac Lab v2.1.1 (simulation, walking policy) plus ROS 2 Humble and the
# dependencies of BIEVR-LIO, elevation_mapping_cupy, TARE and the CMU stack (algorithms). Built by setup.sh.
# Base isaac-lab-base = Isaac Lab v2.1.1 + docker/isaaclab.patch, built with Isaac Lab's own compose file.
#
# Two Pythons side by side, with separate site-packages:
#   Isaac Sim's (/isaac-sim/python.sh, torch 2.7)      -> sim/go2_sim.py and the other Isaac scripts
#   system python3 (torch 2.5.1 cu124 + cupy, pip)     -> the ROS 2 nodes
# cupy needs the pip CUDA libs on LD_LIBRARY_PATH; that is set by docker/algo_env.sh only, never globally,
# because those libs would shadow Isaac Sim's own CUDA libraries.
FROM isaac-lab-base

# setup.sh --cn: Ubuntu / ROS apt mirrors (the base image uses archive.ubuntu.com and packages.ros.org)
ARG UBUNTU_MIRROR=""
ARG ROS_APT_MIRROR=http://packages.ros.org/ros2/ubuntu
ENV DEBIAN_FRONTEND=noninteractive
RUN if [ -n "$UBUNTU_MIRROR" ]; then \
      sed -i -E "s#https?://(archive|security)\.ubuntu\.com#http://$UBUNTU_MIRROR#g" /etc/apt/sources.list; fi && \
    apt-get update && apt-get install -y --no-install-recommends curl gnupg ca-certificates && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] $ROS_APT_MIRROR jammy main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    # the Isaac Sim image has libbrotli1 1.1.0 from a PPA, but Ubuntu's -dev packages (pulled in by rviz) need 1.0.9.
    # Isaac Sim bundles its own brotli, so going back to Ubuntu's version is safe
    apt-get install -y --allow-downgrades "libbrotli1=$(apt-cache madison libbrotli1 | awk 'NR == 1 {print $3}')" && \
    apt-get install -y --no-install-recommends \
      ros-humble-desktop ros-dev-tools ros-humble-rmw-fastrtps-cpp \
      ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-cv-bridge ros-humble-grid-map \
      ros-humble-tf-transformations ros-humble-sensor-msgs-py ros-humble-tf2-geometry-msgs \
      ros-humble-launch-testing-ament-cmake ros-humble-ament-cmake-pytest \
      build-essential cmake git ninja-build libeigen3-dev libgflags-dev libgoogle-glog-dev libsuitesparse-dev \
      libtbb-dev libusb-dev libyaml-cpp-dev qtbase5-dev \
      python3-pip python3-opencv python3-scipy python3-sklearn python3-shapely python3-ruamel.yaml python3-yaml \
      python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# Ceres 2.2.0 from source (the version BIEVR-LIO is tested with). The script compiles with $(nproc) jobs, ~1.5 GB each,
# and docker build has no memory cap: on a 32-core machine that froze the host. nproc honours OMP_NUM_THREADS.
ARG BUILD_JOBS=4
COPY ros2_ws/src/BIEVR-LIO/docker/scripts/install_ceres.sh /tmp/install_ceres.sh
RUN OMP_NUM_THREADS=$BUILD_JOBS bash /tmp/install_ceres.sh && rm -rf /opt/ceres/build /tmp/install_ceres.sh

# Python deps of the ROS 2 nodes, from wheels pre-downloaded by docker/fetch_wheels.sh (pip in the build is slow).
# torch cu124 and the matching nvrtc, which cupy JIT-compiles its kernels with (a newer nvrtc emits PTX that a 550
# driver rejects). numpy<2 for elevation_mapping_cupy; transforms3d>=0.4.2 because apt's 0.3.1 uses np.float.
# PIP_CONSTRAINT (from isaaclab.patch) pins torch 2.7 for Isaac Sim's Python; it must not apply to the system python3.
RUN --mount=type=bind,source=docker/wheels,target=/wheels \
    unset PIP_CONSTRAINT && \
    pip3 install --no-index -f /wheels --ignore-installed "sympy==1.13.1" && \
    pip3 install --no-index -f /wheels "numpy<2" simple-parsing torch==2.5.1 "cupy-cuda12x<14" \
      "nvidia-cuda-nvrtc-cu12==12.4.127" "transforms3d>=0.4.2"

# the project is mounted at /isaac_explore; the Isaac scripts address it as /workspace/isaac_explore
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=/isaac_explore/config/fastdds.xml
RUN ln -s /isaac_explore /workspace/isaac_explore && echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
WORKDIR /isaac_explore
