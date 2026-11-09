#!/bin/bash
# Starts what the Panda giskard demo needs and then the demo itself:
#
#   1. panda_giskard.launch.py: the URDF on /robot_description, the controller
#      manager on /panda/hw/..., and robot_station.hw (robot-station's robot_station
#      package, installed for the ROS python: see below) behind it driving the
#      Panda ($DRIVER: fake, real or lobby);
#   2. waits for the robot's states (/panda/hw/joint_states; DRIVER=lobby: in the
#      lobby, spawn Franka Emika Panda, untick its IK (Teleop panel), switch its
#      ROS on, its bridge a rosbridge_server on this machine's ROS);
#   3. enables the robot (/panda/hw/enable; DRIVER=real asks first, unless YES=1);
#   4. giskard (universal_robot_velocity: any robot with a velocity controller
#      on /controller_manager), until its .../command action is up;
#   5. panda_giskard_demo.py, in the foreground.
#
# Ctrl-C (or the demo ending) stops them all; the robot holds where it is.
# Their logs are in $LOG_DIR.
#
#   DRIVER=fake demo/panda_giskard/run_panda_giskard_demo.sh
#   DRIVER=fake demo/panda_giskard/run_panda_giskard_demo.sh --cycles 0   # arguments: the demo's
#   DRIVER=real IP=172.16.0.2 SPEED=0.2 demo/panda_giskard/run_panda_giskard_demo.sh
#
# Environment: DRIVER (fake), IP (172.16.0.2), SPEED (0.5: of the Panda's joint
# velocity limits), DYNAMICS (0.15: franky's), ROS_WS (a ROS workspace with
# topic_based_ros2_control, its install/; default: this repository's
# ros2_ws/install), LOG_DIR (~/.panda_giskard_demo), WAIT (s to wait for each
# step, 300), YES.
#
# robot_station, for the ROS python (/usr/bin/python3; DRIVER=real: franky too):
#
#   /usr/bin/python3 -m pip install --user --break-system-packages \
#     "git+https://github.com/yxzhan/robot-station"

set -u
HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO_DIR="$(dirname "$(dirname "${HERE}")")"
CRAM="${REPO_DIR}/cognitive_robot_abstract_machine"
PY="${HOME}/.local/bin/cram_python_wrapper.sh"   # ROS 2, ros2_ws and the CRAM venv
LOG_DIR="${LOG_DIR:-${HOME}/.panda_giskard_demo}"
DRIVER="${DRIVER:-fake}"
IP="${IP:-172.16.0.2}"
SPEED="${SPEED:-0.5}"
DYNAMICS="${DYNAMICS:-0.15}"
ROS_WS="${ROS_WS:-${REPO_DIR}/ros2_ws/install}"
/usr/bin/python3 -c 'import robot_station.safety' 2> /dev/null || {
    echo "no robot_station for the ROS python (/usr/bin/python3): install robot-station:"
    echo '  /usr/bin/python3 -m pip install --user --break-system-packages "git+https://github.com/yxzhan/robot-station"'
    exit 1
}
WAIT="${WAIT:-300}"

# ROS for ros2 (its setup files read unset variables: not under set -u).
set +u
source "${ROS_PATH:-/opt/ros/jazzy}/setup.bash"
# topic_based_ros2_control built against today's ros2_control (an older binary
# package can crash the controller manager): $ROS_WS.
if [ -f "${ROS_WS}/setup.bash" ]; then source "${ROS_WS}/setup.bash"
else echo "no ${ROS_WS}/setup.bash (ROS_WS): topic_based_ros2_control from the apt package, if any"; fi
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"

# The demo's arguments checked before anything starts (or the robot is enabled).
"${PY}" "${HERE}/panda_giskard_demo.py" "$@" --check || exit 1

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

start ros2_control ros2 launch "${HERE}/panda_giskard.launch.py" driver:="${DRIVER}" ip:="${IP}" \
    speed:="${SPEED}" dynamics:="${DYNAMICS}"

wait_for "the Panda's states (/panda/hw/joint_states, driver ${DRIVER})" \
    bash -c "timeout 5 ros2 topic echo --once /panda/hw/joint_states sensor_msgs/msg/JointState | grep -q joint1"
wait_for "the controllers" controllers_up /controller_manager

if [ "${DRIVER}" = real ] && [ "${YES:-}" != 1 ]; then
    read -r -p "Enable the real Panda at ${IP} (speed ${SPEED})? E-stop at hand? [y/N] " answer
    [ "${answer}" = y ] || [ "${answer}" = Y ] || exit 1
fi
ros2 service call /panda/hw/enable std_srvs/srv/SetBool "{data: true}" | grep -o "success=.*"
grep -q '"state": "enabled"' <(timeout 5 ros2 topic echo --once /panda/hw/status std_msgs/msg/String) \
    || { echo "not enabled: see /panda/hw/status and ${LOG_DIR}/ros2_control.log"; exit 1; }

# The CRAM venv's python first: giskard's ROS helpers may start `env python3` scripts.
export PATH="${CRAM}/.venv/bin:${PATH}"
start giskard "${PY}" -m giskardpy.middleware.ros2.scripts.tutorial.universal_robot_velocity
wait_for "giskard (its command action)" has_action '/command$'

echo "demo:"
"${PY}" "${HERE}/panda_giskard_demo.py" "$@"
status=$?
echo "demo exited (${status})"
exit "${status}"
