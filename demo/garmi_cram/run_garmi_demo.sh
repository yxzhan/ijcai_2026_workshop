#!/bin/bash
# Starts what the GARMI demo needs and then the demo itself (rosbridge, which
# the lobby's ROS panel connects to, is started by binder/entrypoint.sh):
#
#   1. waits for the lobby's GARMI (its <prefix>/joint_states on ROS): in the
#      lobby, spawn GARMI alone, switch its IK off (Teleop panel) and its ROS on
#      (ROS panel, the rosbridge's address);
#   2. garmi_lobby_bridge.py: the real GARMI's ROS interface on top of it;
#   3. giskard (garmi_velocity), until its .../command action is up;
#   4. garmi_lobby_demo.py, in the foreground.
#
# Ctrl-C (or the demo ending) stops them all. Their logs are in $LOG_DIR.
#
#   demo/garmi_cram/run_garmi_demo.sh [--lobby r1] [--start -4.8 1.5 3.1416]
#
# The options go to garmi_lobby_bridge.py. Environment: LOG_DIR (~/.garmi_demo),
# WAIT (s to wait for each step, 300).

set -u
HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO_DIR="$(dirname "${HERE}")"
PY="${HOME}/.local/bin/cram_python_wrapper.sh"   # ROS 2, ros2_ws and the CRAM venv
LOG_DIR="${LOG_DIR:-${HOME}/.garmi_demo}"
WAIT="${WAIT:-300}"

# The lobby's prefix, from the options the bridge gets (default r1).
LOBBY=r1
args=("$@")
for ((i = 0; i < ${#args[@]}; i++)); do
    [ "${args[i]}" = "--lobby" ] && LOBBY="${args[i + 1]:-r1}"
done

# The same ROS environment as the wrapper, for ros2 (its setup files read unset
# variables: not under set -u).
set +u
source "${ROS_PATH:-/opt/ros/jazzy}/setup.bash"
[ -f "${REPO_DIR}/ros2_ws/install/setup.bash" ] && source "${REPO_DIR}/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"

mkdir -p "${LOG_DIR}"
pids=()
stop() {
    trap - EXIT INT TERM
    echo "stopping: ${pids[*]}"
    for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null; done
    sleep 2
    for p in "${pids[@]}"; do kill -TERM "$p" 2>/dev/null; done
    wait 2>/dev/null
}
trap stop EXIT
trap 'exit 130' INT TERM

# start <name> <command...>: in the background, its output in $LOG_DIR/<name>.log
start() {
    local name=$1; shift
    "$@" > "${LOG_DIR}/${name}.log" 2>&1 &
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

wait_for "the lobby's GARMI (/${LOBBY}/joint_states; spawn GARMI, switch its ROS on)" \
    has_topic "/${LOBBY}/joint_states"

start bridge "${PY}" "${HERE}/garmi_lobby_bridge.py" "$@"
wait_for "the bridge (/garmi/joint_states)" has_topic /garmi/joint_states

start giskard "${PY}" -m giskardpy.middleware.ros2.scripts.other_robots.garmi.garmi_velocity
wait_for "giskard (its command action)" has_action '/command$'

echo "demo:"
"${PY}" "${HERE}/garmi_lobby_demo.py"
status=$?
echo "demo exited (${status})"
exit "${status}"
