# ROS 1 Noetic environment for the GBPlanner2 baseline. Only the environment: the sources (fetch_src.sh -> src/) are
# mounted at run time and built with catkin by ros1.sh build. Build context: ros1_ws.
FROM ros:noetic-ros-base

ARG UBUNTU_MIRROR=""
ARG ROS_APT_MIRROR=""
ENV DEBIAN_FRONTEND=noninteractive
# the installs run up to 3 times: a flaky mirror or proxy fails some downloads (502 Bad Gateway, which focal's apt
# does not retry); the packages already downloaded are kept, a retry fetches only the rest
RUN if [ -n "$UBUNTU_MIRROR" ]; then \
      sed -i -E "s#https?://(archive|security)\.ubuntu\.com#http://$UBUNTU_MIRROR#g" /etc/apt/sources.list; fi && \
    if [ -n "$ROS_APT_MIRROR" ]; then \
      sed -i -E "s#https?://packages\.ros\.org/ros/ubuntu#$ROS_APT_MIRROR#g" /etc/apt/sources.list.d/ros*.list; fi && \
    apt-get update && for i in 1 2 3; do apt-get install -y --no-install-recommends \
      python3-catkin-tools python3-osrf-pycommon git autoconf automake libtool \
      libprotobuf-dev protobuf-compiler libeigen3-dev libgoogle-glog-dev \
      ros-noetic-rviz ros-noetic-topic-tools && break; done \
    && rm -rf /var/lib/apt/lists/*

# everything else the fetched packages declare in their package.xml files. rosdep runs apt-get once per package and the
# image's docker-clean hook deletes the downloaded archives after each: off for this step, so a retry fetches only the rest
RUN --mount=type=bind,source=src,target=/src \
    mv /etc/apt/apt.conf.d/docker-clean /tmp/ && apt-get update && rosdep update --rosdistro noetic && \
    for i in 1 2 3; do rosdep install --from-paths /src --ignore-src -y -r --rosdistro noetic && break; done && \
    mv /tmp/docker-clean /etc/apt/apt.conf.d/ && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "[ -f /ws/devel/setup.bash ] && source /ws/devel/setup.bash" >> /root/.bashrc
WORKDIR /ws
