# Robot Lobby robots driven by ROS 2 and CRAM

Demos that drive robots of [Robot Lobby](https://yxzhan.github.io/robot-lobby/), a MuJoCo world
in the browser, as if they were the real robots: the page's ROS panel connects to the rosbridge
of this environment (port 9090, started with it by `binder/entrypoint.sh`), and the programs
here talk to them as to the real ones.

[![Binder](https://binder.intel4coro.de/badge_logo.svg)](https://binder.intel4coro.de/v2/gh/yxzhan/ijcai_2026_workshop/HEAD?urlpath=lab%2Ftree%2Fdemo)

## What is in here

- `demo/garmi_cram/`: [CRAM](https://github.com/cram2/cognitive_robot_abstract_machine)'s
  GARMI apartment demo (coraplex, giskardpy) on the lobby's GARMI.
  - `garmi_lobby_bridge.py`: the real GARMI's ROS interface on top of the lobby's ROS topics
    (joint states, base odometry, the arms' velocity commands, franka_gripper's actions for the
    hands), so giskardpy's `garmi_velocity` and coraplex's `RealGarmiPlanExecutor` drive it
    unchanged;
  - `garmi_lobby_demo.py`: coraplex's GARMI demo (a bowl and a spoon carried across the
    apartment) on that executor;
  - `run_garmi_demo.sh`: the bridge, giskard and the demo in one go.
- `demo/tracy_cram/run_tracy_demo.sh`: CRAM's own Tracy demo (`coraplex_real_tracy`) as on the
  real robot, against giskard in standalone mode (it simulates Tracy itself; no lobby needed).
- `demo/tracy_roscontrol/`: the lobby's Tracy under ros2_control as the real one (a controller
  manager per arm), driven by giskard's `tracy_velocity` and CRAM's Tracy demo.
- `demo/fr3_roscontrol/`: the lobby's Franka FR3 with its hand under ros2_control, as
  franka_ros2 runs the real one, with the lobby as the hardware (`topic_based_ros2_control`).
  - `fr3.urdf`: its kinematic tree with the `<ros2_control>` block, made by `mjcf2urdf.py` from
    the lobby's MJCF (in a robot-lobby checkout; see its docstring);
  - `controllers.yaml`: joint state broadcaster, a joint trajectory controller for the arm
    (`/fr3_arm_controller/follow_joint_trajectory`), a parallel gripper controller for the hand
    (`/fr3_gripper/gripper_cmd`);
  - `fr3_lobby.launch.py`: robot_state_publisher, the controller manager and the spawners
    (`rosbridge:=true` for a rosbridge of its own, without the entrypoint's);
  - `demo.py`: a trajectory and gripper client as one would write for the real robot.
- `binder/`: the image: ROS 2 Jazzy, the CRAM workspace (its venv and its ROS workspace
  `ros2_ws`), and in `ros2_ws` also `franka_msgs` and `topic_based_ros2_control`, with
  rosbridge and ros2_control.

In the lobby's ROS panel, the bridge's address: on Binder its proxy path,
`wss://<hub>/user/<name>/proxy/9090/`; locally `localhost:9090`.

## GARMI and CRAM

```bash
demo/garmi_cram/run_garmi_demo.sh          # [--lobby r1] [--start -4.8 1.5 3.1416]
```

waits for the lobby's GARMI to appear on ROS, starts the bridge and giskard, and runs the demo
once giskard is up; Ctrl-C stops them all, their logs are in `~/.garmi_demo/`. In the lobby
meanwhile: spawn GARMI alone, switch its IK off (Teleop panel) and its ROS on (ROS panel).

By hand, the same in three terminals, each with the CRAM kernel's environment
(`~/.local/bin/cram_python_wrapper.sh` instead of `python`: ROS 2, `ros2_ws` and the CRAM venv):

```bash
~/.local/bin/cram_python_wrapper.sh demo/garmi_cram/garmi_lobby_bridge.py
~/.local/bin/cram_python_wrapper.sh -m giskardpy.middleware.ros2.scripts.other_robots.garmi.garmi_velocity
~/.local/bin/cram_python_wrapper.sh demo/garmi_cram/garmi_lobby_demo.py
```

Each script's docstring has the details (`--lobby r1`, `--start x y yaw`).

## Tracy and CRAM (giskard standalone)

```bash
demo/tracy_cram/run_tracy_demo.sh
```

The demo starts giskard itself (`ros2 launch giskardpy_ros giskardpy_tracy_standalone.launch.py`),
fetches its world over ROS and sends it the plan: park, then pick up two boxes and stack them on
the third. The script puts the CRAM venv first on `PATH`: the nodes the launch file starts run
`#!/usr/bin/env python3`, which would otherwise be conda's python, without giskardpy
(`ModuleNotFoundError: No module named 'giskardpy'`).

Known (CRAM 2026-11): giskard runs the plan (`Goal succeeded` in its log), but the demo can then
hang: its copy of the world loses updates from giskard (`Updates #… were lost`, then
`rustworkx.NoEdgeBetweenNodes`), which stops its ROS thread, so the goal's result never arrives.
Ctrl-C ends the demo and its giskard.

## Tracy on the lobby (ros2_control, giskard tracy_velocity)

```bash
demo/tracy_roscontrol/run_tracy_lobby_demo.sh
# in the lobby: spawn Tracy, untick its IK (Teleop panel), switch its ROS on
```

The lobby's Tracy as the real one's hardware: `tracy_lobby.launch.py` puts iai_tracy's URDF
(`iai_tracy_description`, in `ros2_ws`) on `/robot_description` and runs a controller manager
per arm (`/left_arm/controller_manager`, `/right_arm/controller_manager`, each a joint state
broadcaster and a `forward_velocity_controller`) on `topic_based_ros2_control`, on the lobby's
`/robot_joint_states`. The lobby takes position targets only, so `tracy_lobby_relay.py`
integrates the velocity commands into `/robot_joint_commands`, and republishes the grippers'
joints on `/left_gripper/joint_states`, `/right_gripper/joint_states`. Then giskard
(`tracy_velocity`) and `tracy_lobby_demo.py`: CRAM's Tracy demo without its standalone giskard.
The boxes are in giskard's world only. Logs in `~/.tracy_demo/`.

Known (CRAM 2026-11): giskard moves the arms, but the demo's copy of the world loses bursts of
giskard's updates (`Updates #… were lost`), and the demo stops with
`GiskardWorldUpdateNotReceivedError` 30 s into the first goal, as in standalone mode.
Nothing in the lobby or ros2_control is involved.

## FR3 and ros2_control

```bash
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
ros2 launch demo/fr3_roscontrol/fr3_lobby.launch.py
# in the lobby: spawn Franka FR3 with hand, untick its IK (Teleop panel), switch its ROS on
python3 demo/fr3_roscontrol/demo.py      # home, a reach, close, open, home
```

`use_sim_time:=true` with the lobby's Clock and simulated stamps on (its ROS panel).

## Running it locally

```bash
git clone --recursive https://github.com/yxzhan/ijcai_2026_workshop.git
cd ijcai_2026_workshop
docker compose -f binder/docker-compose.yml up --build
```

Then open <http://localhost:8888>.
