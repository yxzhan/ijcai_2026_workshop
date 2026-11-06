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
- `demo/fr3_giskard/`: the lobby's FR3 moved by giskardpy alone (no CRAM plans): a joint goal,
  a Cartesian goal for the hand, its gripper action.
- `demo/panda_giskard/`: a Franka Emika Panda (FER, Panda System 4.2.x) moved by giskardpy, the
  same program for the real robot, a fake one and the lobby's: robot-lobby's
  `real_robot.ros2` as the ros2_control hardware behind `topic_based_ros2_control`.
- `demo/lobby_velocity_relay.py`: velocity commands of ros2_control as the lobby's position
  targets (the lobby takes positions only), for the Tracy and FR3 giskard demos.
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
`/robot_joint_states`. The lobby takes position targets only, so `demo/lobby_velocity_relay.py`
integrates the velocity commands into `/robot_joint_commands`, and republishes the grippers'
joints on `/left_gripper/joint_states`, `/right_gripper/joint_states`. Then giskard
(`tracy_velocity`) and `tracy_lobby_demo.py`: CRAM's Tracy demo without its standalone giskard.
The boxes are in giskard's world only. Logs in `~/.tracy_demo/`.

Known (CRAM 2026-11): giskard moves the arms, but the demo's copy of the world loses bursts of
giskard's updates (`Updates #… were lost`), and the demo stops with
`GiskardWorldUpdateNotReceivedError` 30 s into the first goal, as in standalone mode.
Nothing in the lobby or ros2_control is involved.

## FR3 and giskardpy

```bash
demo/fr3_giskard/run_fr3_giskard_demo.sh
# in the lobby: spawn Franka FR3 with hand, untick its IK (Teleop panel), switch its ROS on
```

giskardpy without CRAM's plans. `fr3_giskard.launch.py`: `../fr3_roscontrol/fr3.urdf` with the
arm velocity commanded and FR3's velocity limits, the controller manager on
`topic_based_ros2_control` (a `forward_velocity_controller` for the arm, `fr3_gripper` for the
hand) and `demo/lobby_velocity_relay.py`. giskard is its generic
`giskardpy.middleware.ros2.scripts.tutorial.universal_robot_velocity` (any robot whose
`/controller_manager` runs a velocity controller, its URDF on `/robot_description`).
`fr3_giskard_demo.py` sends it a joint goal (Franka's ready pose) and a `CartesianPose` for
`fr3_hand`, and moves the hand with its gripper action. Logs in `~/.fr3_giskard_demo/`.

ros2_control reads the arm's states from the relay (`--state-out`): its integrated targets,
not the lobby's measurements. Through the network to the browser and back, those come a few
hundred ms late, and giskard, closing its loop on them, overshoots and oscillates without
reaching a goal. The lobby follows the targets; when the arm is idle and the lobby's arm is
more than 0.1 rad from them (moved in the lobby), they are reset to it. The Tracy demo does the same.

## Panda and giskardpy (real, fake or the lobby's)

```bash
DRIVER=fake demo/panda_giskard/run_panda_giskard_demo.sh       # no hardware
DRIVER=lobby demo/panda_giskard/run_panda_giskard_demo.sh      # the lobby's Panda, over rosbridge
DRIVER=real IP=172.16.0.2 SPEED=0.2 demo/panda_giskard/run_panda_giskard_demo.sh
```

The FR3 demo's program for a Panda: a joint goal, a `CartesianPose` for `hand`, the gripper
closed and opened, the ready pose again. `panda_giskard.launch.py`: `panda.urdf` (made by
`mjcf2urdf.py` from the lobby's `mujoco_menagerie/franka_emika_panda/panda.xml`: `joint1..7`,
`finger_joint1`, `link0`..`hand`; no collision geometry) with the arm velocity commanded at
`SPEED` of the Panda's velocity limits, the controller manager on `topic_based_ros2_control`
(`/panda/hw/...`) and a `forward_velocity_controller`; giskard is `universal_robot_velocity`
as for the FR3.

Behind the topics, robot-lobby's `python3 -m real_robot.ros2` (`LOBBY_DIR`: a robot-lobby
checkout; by default the directory this repository sits in) integrates the velocities into
position commands for its driver, through its safety layer (joint limits, `SPEED`, commands
older than 0.3 s hold the robot, a fault when the robot falls 0.5 rad behind), and reports the
commanded positions to ros2_control (giskard closes its loop on them, as with the lobby relay),
the measured ones on `/panda/hw/measured_joint_states`. The gripper is its action
`/panda/hw/gripper_cmd` (`control_msgs/ParallelGripperCommand`, `finger_joint1`: each finger's
opening, 0-0.04 m), not a controller. The robot holds, disabled, until `/panda/hw/enable`
(`std_srvs/SetBool`); the run script enables it, for `DRIVER=real` after asking (`YES=1`: not).

- `DRIVER=real`: franky on libfranka 0.9.x (Panda System 4.2.x; libfranka >= 0.10 and
  franka_ros2 are for the FR3), installed for the ROS python (its libfranka 0.9.2 wheels, see
  robot-lobby's `real_robot/README.md`); a PREEMPT_RT kernel; FCI active in Desk. Each command
  becomes a franky `JointMotion` (Ruckig, `DYNAMICS`: its `relative_dynamics_factor`).
- `DRIVER=lobby`: in the lobby, spawn Franka Emika Panda, untick its IK (Teleop panel), switch
  its ROS on (its profile's prefix `panda`: `/panda/joint_states`, `/panda/joint_command`),
  its bridge a `rosbridge_server` on this ROS.

Logs in `~/.panda_giskard_demo/`. Needs `topic_based_ros2_control` built against the installed
ros2_control (this repository's `ros2_ws`, else robot-lobby's `ros_ws`), and matching
ros2_control packages: a `ros-jazzy-controller-manager` newer than `ros-jazzy-hardware-interface`
crashes the controller manager on start (segfault in `load_and_initialize_components`; the FR3
demos too): `apt-get install --only-upgrade ros-jazzy-hardware-interface ros-jazzy-controller-interface`.

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
