#!/bin/bash
# CRAM's Tracy demo (coraplex/demos/coraplex_real_tracy/demo.py) as on the real
# robot, against giskard in standalone mode (it simulates Tracy itself; no lobby
# and no robot needed): the demo starts that giskard itself (ros2 launch
# giskardpy_ros giskardpy_tracy_standalone.launch.py), fetches its world over ROS
# and sends it the plan (park, pick up, place two boxes onto a third).
#
#   demo/tracy_cram/run_tracy_demo.sh
#
# What it needs beyond the CRAM kernel's environment: the CRAM venv's python as
# `python3` for the nodes the launch file starts (their `#!/usr/bin/env python3`
# would be conda's, without giskardpy). Ctrl-C stops the demo and its giskard.
#
# Known (CRAM 2026-11): giskard runs the plan ("Goal succeeded"), but the demo may
# then hang: its world synchronizer drops an update from giskard
# (rustworkx.NoEdgeBetweenNodes after "Updates #... were lost") and stops its ROS
# thread, so the goal's result never arrives. Ctrl-C ends it.

set -u
HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO_DIR="$(dirname "$(dirname "${HERE}")")"
CRAM="${REPO_DIR}/cognitive_robot_abstract_machine"
PY="${HOME}/.local/bin/cram_python_wrapper.sh"   # ROS 2, ros2_ws and the CRAM venv

export PATH="${CRAM}/.venv/bin:${PATH}"
cd "${REPO_DIR}"
exec "${PY}" "${CRAM}/coraplex/demos/coraplex_real_tracy/demo.py"
