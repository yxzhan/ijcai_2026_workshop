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
