#!/bin/bash
# Starts what the Tracy demo on the lobby needs and then the demo itself
# (rosbridge, which the lobby's ROS panel connects to, is started by
# binder/entrypoint.sh):
#
#   1. tracy_lobby.launch.py: the URDF on /robot_description, a controller
#      manager per arm on the lobby's topics;
#   2. waits for the lobby's Tracy (/robot_joint_states): in the lobby, spawn
#      Tracy, untick its IK (Teleop panel), switch its ROS on (ROS panel);
#   3. giskard (tracy_velocity), until its .../command action is up;
#   4. tracy_lobby_demo.py, in the foreground.
#
# Ctrl-C (or the demo ending) stops them all. Their logs are in $LOG_DIR.
#
#   demo/tracy_roscontrol/run_tracy_lobby_demo.sh
#
# Environment: LOG_DIR (~/.tracy_demo), WAIT (s to wait for each step, 300).

set -u
HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO_DIR="$(dirname "$(dirname "${HERE}")")"
CRAM="${REPO_DIR}/cognitive_robot_abstract_machine"
PY="${HOME}/.local/bin/cram_python_wrapper.sh"   # ROS 2, ros2_ws and the CRAM venv
LOG_DIR="${LOG_DIR:-${HOME}/.tracy_demo}"
WAIT="${WAIT:-300}"

# ROS for ros2 (its setup files read unset variables: not under set -u).
set +u
source "${ROS_PATH:-/opt/ros/jazzy}/setup.bash"
source "${REPO_DIR}/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"

mkdir -p "${LOG_DIR}"
pids=()
stop() {
    trap - EXIT INT TERM
    echo "stopping: ${pids[*]}"
    # Each runs in a process group of its own (start): signal the group, so the
    # nodes a launch file started stop too.
    for p in "${pids[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
    for _ in $(seq 10); do
        alive=0
        for p in "${pids[@]}"; do kill -0 -- "-$p" 2>/dev/null && alive=1; done
        [ "${alive}" = 0 ] && break
        sleep 1
    done
    for p in "${pids[@]}"; do kill -TERM -- "-$p" 2>/dev/null; done
    sleep 1
    for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
    wait 2>/dev/null
}
trap stop EXIT
trap 'exit 130' INT TERM

# start <name> <command...>: in the background, in a process group of its own, its
# output in $LOG_DIR/<name>.log
start() {
    local name=$1; shift
    setsid "$@" > "${LOG_DIR}/${name}.log" 2>&1 < /dev/null &
    pids+=($!)
    echo "${name}: pid $! (log ${LOG_DIR}/${name}.log)"
}

# wait_for <what> <test command...>: until the test succeeds, or WAIT s; fails if a
# process started above has died meanwhile.
wait_for() {
    local what=$1; shift
    local t0=$SECONDS
    echo "waiting for ${what}..."
    until "$@" > /dev/null 2>&1; do
        for p in "${pids[@]}"; do
            kill -0 "$p" 2>/dev/null || { echo "a process died (pid $p): see ${LOG_DIR}"; exit 1; }
        done
        if (( SECONDS - t0 > WAIT )); then echo "no ${what} after ${WAIT} s"; exit 1; fi
        sleep 1
    done
    echo "${what}: ok"
}

has_topic() { ros2 topic list | grep -qx "$1"; }
has_action() { ros2 action list | grep -q "$1"; }
# A controller manager's controllers, all active.
controllers_up() {
    local out
    out=$(timeout 15 ros2 control list_controllers -c "$1" 2>/dev/null) || return 1
    [ "$(grep -c ' active' <<< "${out}")" -ge 2 ]
}

start ros2_control ros2 launch "${HERE}/tracy_lobby.launch.py"

wait_for "the lobby's Tracy (/robot_joint_states; spawn Tracy, untick its IK, switch its ROS on)" \
    bash -c "timeout 5 ros2 topic echo --once /robot_joint_states sensor_msgs/msg/JointState | grep -q left_shoulder_pan_joint"
wait_for "the left arm's controllers" controllers_up /left_arm/controller_manager
wait_for "the right arm's controllers" controllers_up /right_arm/controller_manager

# The CRAM venv's python first: giskard's ROS helpers may start `env python3` scripts.
export PATH="${CRAM}/.venv/bin:${PATH}"
start giskard "${PY}" -m giskardpy.middleware.ros2.scripts.iai_robots.tracy.tracy_velocity
wait_for "giskard (its command action)" has_action '/command$'

echo "demo:"
"${PY}" "${HERE}/tracy_lobby_demo.py"
status=$?
echo "demo exited (${status})"
exit "${status}"
