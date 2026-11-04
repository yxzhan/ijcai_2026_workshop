# Dockerfile of the base image: https://github.com/IntEL4CoRo/jupyter-ros2
FROM intel4coro/jupyter-ros2:jazzy-py3.12

# Install UV (builds the giskard venv below)
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
#===========================================================#

# VSCode extension to view the simulation desktop from code-server
RUN curl -LO https://github.com/yxzhan/vscode_remote_desktop/releases/download/v0.1.0/vscode-remote-desktop-0.1.0.vsix && \
    code-server --install-extension vscode-remote-desktop-0.1.0.vsix && \
    rm vscode-remote-desktop-0.1.0.vsix

# Python packages for the jupyter env (rarely changes).
USER ${NB_USER}
COPY --chown=${NB_USER}:users binder/requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

ENV REPO_DIR=${HOME}/ijcai_2026_workshop
RUN mkdir -p ${REPO_DIR}
COPY --chown=${NB_USER}:users cognitive_robot_abstract_machine ${REPO_DIR}/cognitive_robot_abstract_machine
RUN git config --global --add safe.directory ${REPO_DIR}/cognitive_robot_abstract_machine

RUN cd ${REPO_DIR}/cognitive_robot_abstract_machine && \
    ~/.local/bin/uv sync && \
    .venv/bin/pip install nest_asyncio argcomplete usd-core

RUN sudo apt-get update && \
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends git-lfs && \
    git lfs install && \
    OVERLAY_WS=${REPO_DIR}/ros2_ws \
        bash ${REPO_DIR}/cognitive_robot_abstract_machine/.github/docker/setup_ros_workspace.sh && \
    sudo rm -rf /var/lib/apt/lists/*

#============= Robot Lobby's ROS packages (GARMI demo, FR3) =============#
# Into the same workspace, at the versions robot-lobby's ros_ws/ has them:
#   - franka_msgs (franka_ros2 v3.6.1, that package alone: the rest needs
#     libfranka): franka_gripper's move / grasp / homing actions, which
#     notebooks/garmi_lobby_bridge.py serves for GARMI's hands;
#   - topic_based_ros2_control, built against this image's ros2_control (its
#     binary package was built against an older one and crashes loading the
#     hardware): the lobby's FR3 under ros2_control.
# The apt packages: rosbridge (the lobby's ROS panel connects to it, port
# 9090), ros2_control and control_msgs (MultiDOFCommand, the arms' velocity
# commands) and what topic_based_ros2_control builds against.
RUN sudo apt-get update && \
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-jazzy-rosbridge-suite \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-control-msgs \
        ros-jazzy-parallel-gripper-controller \
        ros-jazzy-angles && \
    sudo rm -rf /var/lib/apt/lists/* && \
    cd ${REPO_DIR}/ros2_ws/src && \
    git clone --filter=blob:none --no-checkout https://github.com/frankaemika/franka_ros2.git && \
    git -C franka_ros2 sparse-checkout set franka_msgs && \
    git -C franka_ros2 checkout 8c946641bdedaf20c35604d8bc5a809117e4ed6c && \
    git clone https://github.com/PickNikRobotics/topic_based_ros2_control.git && \
    git -C topic_based_ros2_control checkout aa9f9ef9525a60f8becea322537dd1d6d1bf9123 && \
    cd ${REPO_DIR}/ros2_ws && \
    bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
        colcon build --merge-install --packages-select franka_msgs topic_based_ros2_control \
            --cmake-args -DBUILD_TESTING=OFF" && \
    rm -rf build log

#============= Fixes on top of the cached layers above =============#
# Kept here, after the expensive layers (the CRAM venv, its ROS workspace, the
# packages above), so that the build reuses them.
# - The base image's ROS (2025) upgraded to the current Jazzy snapshot: the
#   packages installed above come from that snapshot and are built against
#   newer core libraries; on the old ones rosbridge_msgs fails to load
#   (undefined symbol eprosima::fastcdr::Cdr::serialize, fastcdr 2.2.5). The
#   CRAM workspace built above runs on the upgraded ones; franka_msgs and
#   topic_based_ros2_control are built again on them (topic_based_ros2_control
#   as built before the upgrade crashed ros2_control_node on loading the FR3's
#   hardware: segmentation fault in TopicBasedSystem::export_state_interfaces).
# - six into the CRAM venv: krrood (ripple_down_rules) imports it.
RUN sudo apt-get update && \
    sudo DEBIAN_FRONTEND=noninteractive apt-get install --only-upgrade -y \
        $(apt list --upgradable 2>/dev/null | grep -o '^ros-jazzy-[^/]*') && \
    sudo rm -rf /var/lib/apt/lists/* && \
    cd ${REPO_DIR}/ros2_ws && \
    bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
        PATH=/usr/bin:\$PATH colcon build --merge-install --packages-select franka_msgs topic_based_ros2_control \
            --cmake-args -DBUILD_TESTING=OFF" && \
    rm -rf build log && \
    ${REPO_DIR}/cognitive_robot_abstract_machine/.venv/bin/pip install six

COPY --chown=${NB_USER}:users binder/cram_python_wrapper.sh /home/jovyan/.local/bin/cram_python_wrapper.sh
COPY --chown=${NB_USER}:users binder/cram-kernel.json /opt/conda/share/jupyter/kernels/cram/kernel.json
COPY --chown=${NB_USER}:users binder ${REPO_DIR}/binder
COPY --chown=${NB_USER}:users demo ${REPO_DIR}/demo

RUN git config --global --add safe.directory ${REPO_DIR}

# Set Jupyterlab default directory
WORKDIR ${REPO_DIR}

# Set VSCODE default directory
ENV CODE_WORKING_DIRECTORY=${REPO_DIR}

# The entrypoint of the docker image
COPY --chown=${NB_USER}:users binder/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
