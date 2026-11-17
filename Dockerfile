# syntax=docker/dockerfile:1.7
# ASU-FR3: one image for the CRISP controllers on the Franka FR3 (MuJoCo, Isaac, real arm).
# Derived from crisp_controllers_demos (MIT) and franka_ros2 (Apache-2.0); see NOTICE.
#
# Stages exist only so slow steps stay cached. Only the last stage ("final") is shipped:
#   base   -> ROS + apt + MuJoCo + user          (rarely changes)
#   franka -> libfranka, franka_ros2, description (30-60 min)
#   crisp  -> crisp_controllers, MuJoCo plugin    (10-15 min)
#   tools  -> extra apt/pip tools                 (minutes)
#   final  -> our own launch packages             (1-3 min)
# Build up to a stage with:  docker build --target crisp .
#
# Every version below is pinned (image digest, git commit, tarball hash). Values come from the
# fingerprint of the working crisp_controllers_demos:franka-overlay image (archive/tests/baseline_fingerprint).

ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=osrf/ros:jazzy-desktop@sha256:f9f1d1ec0e37b4bf8eec70b70ed5f716293a238c412beefc44249d4e08dadb1e

# ---------------------------------------------------------------- base
FROM ${BASE_IMAGE} AS base

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ARG MUJOCO_VERSION=3.2.6
ARG MUJOCO_SHA256=68936e0e74a6ef4b0e4efb43efeaa0a63fbe456384e548b4f7396875beef0926

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Replace any existing user/group with our uid/gid, then create the non-root user.
RUN if getent passwd ${USER_UID}; then userdel -r $(getent passwd ${USER_UID} | cut -d: -f1); fi \
 && if getent group ${USER_GID}; then groupdel $(getent group ${USER_GID} | cut -d: -f1); fi \
 && groupadd --gid $USER_GID $USERNAME \
 && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
 && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

RUN apt-get update \
 && apt-get install -y sudo \
 && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    vim build-essential cmake wget git unzip pip python3-venv python3-flake8 \
    python3-rosdep python3-setuptools python3-vcstool python3-colcon-common-extensions \
    libpoco-dev libeigen3-dev \
    ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp ros-$ROS_DISTRO-rmw-zenoh-cpp \
    dpkg \
 && rm -rf /var/lib/apt/lists/* \
 && ln -s /usr/bin/python3 /usr/bin/python

USER $USERNAME
RUN mkdir -p /home/ros/ros2_ws
WORKDIR /home/ros

ENV MUJOCO_VERSION=${MUJOCO_VERSION}
RUN sudo apt-get update && sudo apt-get install -y libglfw3-dev wget \
 && wget -q https://github.com/google-deepmind/mujoco/releases/download/$MUJOCO_VERSION/mujoco-$MUJOCO_VERSION-linux-x86_64.tar.gz \
 && echo "${MUJOCO_SHA256}  mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz" | sha256sum -c - \
 && tar -xzf mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz -C /home/ros \
 && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /home/ros/ros2_ws

# ---------------------------------------------------------------- franka
FROM base AS franka

ARG FRANKA_ROS2_REF=ad6631e0f066b620886949e729e2f25118ed5d7e
ARG FRANKA_DESCRIPTION_REF=68a3288ac16e5b2149777b3c241e5743ac8e06ce
ARG LIBFRANKA_REF=b9091305cbfa9bb3d106c6dbfa58f1aa880d9e4d

RUN git clone https://github.com/frankarobotics/franka_ros2.git src/franka_ros2 \
 && git -C src/franka_ros2 checkout ${FRANKA_ROS2_REF} \
 && source /opt/ros/$ROS_DISTRO/setup.bash \
 && sudo apt-get update \
 && vcs import src < src/franka_ros2/franka.repos --recursive --skip-existing \
 && git -C src/franka_description fetch --tags \
 && git -C src/franka_description checkout ${FRANKA_DESCRIPTION_REF} \
 && git -C src/libfranka fetch --tags \
 && git -C src/libfranka checkout ${LIBFRANKA_REF} \
 && git -C src/libfranka submodule update --init --recursive \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
 && touch src/franka_ros2/franka_gazebo/COLCON_IGNORE \
 && MAKEFLAGS=-j3 CMAKE_BUILD_PARALLEL_LEVEL=3 colcon build --symlink-install --parallel-workers 2 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && touch src/franka_ros2/COLCON_IGNORE src/libfranka/COLCON_IGNORE src/franka_description/COLCON_IGNORE \
 && sudo rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------- crisp
FROM franka AS crisp

ARG CRISP_CONTROLLERS_REF=ccbead4ba22b1a5b43ac8e81d7137c9ed7b65925
ARG FRANKA_BROADCASTERS_REF=f148ae178c94e7b7037fdb7e7044e1c70a92aa80

COPY --chown=ros:ros packages/crisp_mujoco_sim src/crisp_mujoco_sim
RUN git clone https://github.com/utiasDSL/crisp_controllers.git src/crisp_controllers \
 && git -C src/crisp_controllers checkout ${CRISP_CONTROLLERS_REF} \
 && git clone https://github.com/danielsanjosepro/franka_broadcasters.git src/franka_broadcasters \
 && git -C src/franka_broadcasters checkout ${FRANKA_BROADCASTERS_REF}

RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && source /home/ros/ros2_ws/install/setup.bash \
 && sudo apt-get update \
 && rosdep update \
 && rosdep install -q --from-paths src --ignore-src -y \
 && colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      --packages-select crisp_controllers crisp_mujoco_sim franka_force_feedback_controllers \
 && sudo rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------- tools
FROM crisp AS tools

# Additions beyond the baseline image (also listed in archive/tests/allowed_additions.txt).
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-joint-state-topic-hardware-interface \
    rt-tests \
 && sudo rm -rf /var/lib/apt/lists/*

# MuJoCo python bindings, used only by the Isaac stand-in bridge (asu_fr3_bringup.mujoco_isaac_bridge).
RUN sudo pip3 install --break-system-packages --no-cache-dir mujoco==3.2.6

# CRISP Python client and gym environments, core only (no LeRobot, no torch), at the versions of the pixi env used so far.
# SciPy comes from apt and pip cannot replace it, so the newer one crisp_gym needs is installed beside it.
ARG CRISP_PY_VERSION=3.6.0
ARG CRISP_GYM_REF=ab002ba3dfd36c3f00ab46a7d85f4dce4e213b53
RUN echo "numpy==1.26.4" > /tmp/constraints.txt \
 && sudo pip3 install --break-system-packages --no-cache-dir -c /tmp/constraints.txt --ignore-installed "scipy>=1.15.2,<2" \
 && sudo pip3 install --break-system-packages --no-cache-dir -c /tmp/constraints.txt \
      "crisp-python==${CRISP_PY_VERSION}" "git+https://github.com/utiasDSL/crisp_gym@${CRISP_GYM_REF}"

# VR teleop link: Unity's ros_tcp_endpoint (headset TCP bridge, tag ROS2v0.7.0) and the Quest2ROS message package
# (OVR2ROSInputs ...), the same two commits the old stack built in pixi_franka_ros2-main/src.
ARG ROS_TCP_ENDPOINT_REF=54c1a64b6d5ef6ffa0a0431570bb74329b79b15b
ARG QUEST2ROS_REF=6e9e8afec0e00da4da9e6650cfe69e21bb500aad
RUN git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git src/ros_tcp_endpoint \
 && git -C src/ros_tcp_endpoint checkout ${ROS_TCP_ENDPOINT_REF} \
 && git clone https://github.com/Quest2ROS/quest2ros.git src/quest2ros \
 && git -C src/quest2ros checkout ${QUEST2ROS_REF} \
 && source /opt/ros/$ROS_DISTRO/setup.bash \
 && source /home/ros/ros2_ws/install/setup.bash \
 && colcon build --symlink-install --packages-select ros_tcp_endpoint quest2ros \
 && touch src/ros_tcp_endpoint/COLCON_IGNORE src/quest2ros/COLCON_IGNORE

# ---------------------------------------------------------------- final
FROM tools AS final

# Our launch + adapter package (copied from crisp_controllers_demos, renamed in M3.2).
COPY --chown=ros:ros packages/asu_fr3_bringup src/asu_fr3_bringup
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && source /home/ros/ros2_ws/install/setup.bash \
 && sudo apt-get update \
 && rosdep install -q --from-paths src/asu_fr3_bringup --ignore-src -y \
 && colcon build --symlink-install --packages-select asu_fr3_bringup \
 && sudo rm -rf /var/lib/apt/lists/*

# Baked defaults. From M3.3 the launch reads these (an override mounted at /ws_override wins).
ENV ASU_FR3_DIR=/opt/asu_fr3
COPY --chown=ros:ros config /opt/asu_fr3/config
COPY --chown=ros:ros models /opt/asu_fr3/models
COPY --chown=ros:ros scripts /opt/asu_fr3/scripts
COPY --chown=ros:ros tools /opt/asu_fr3/tools
COPY --chown=ros:ros examples /opt/asu_fr3/examples
COPY --chown=ros:ros --chmod=0755 bin /opt/asu_fr3/bin

# Entrypoint: sources ROS and the workspace, picks the DDS implementation (RMW=cyclone|fastdds), runs the command.
COPY --chmod=0755 docker/entrypoint.sh /usr/local/bin/asu-fr3-entrypoint
ENTRYPOINT ["/usr/local/bin/asu-fr3-entrypoint"]

LABEL org.opencontainers.image.title="asu-fr3" \
      org.opencontainers.image.source="https://github.com/mohamedmsamy/ASU-FR3" \
      org.opencontainers.image.licenses="Apache-2.0"

CMD ["/bin/bash"]
