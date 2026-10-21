# Environment of the algorithm processes inside the container: ROS 2 + this workspace, plus the pip CUDA libraries that
# cupy (elevation_mapping_cupy) loads. Sourced by explore.sh and run_trial.sh; by hand in a ./container.sh shell.
# Never source it for the Isaac Sim process: these CUDA libraries would shadow Isaac Sim's own.
source /opt/ros/humble/setup.bash
[ -f /isaac_explore/ros2_ws/install/setup.bash ] && source /isaac_explore/ros2_ws/install/setup.bash
NV=/usr/local/lib/python3.10/dist-packages/nvidia
export LD_LIBRARY_PATH=$NV/cuda_nvrtc/lib:$NV/cuda_runtime/lib:$NV/cublas/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
