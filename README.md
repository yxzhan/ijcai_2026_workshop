# GARMI in Robot Lobby, driven by CRAM

[CRAM](https://github.com/cram2/cognitive_robot_abstract_machine)'s GARMI apartment demo
(coraplex, giskardpy) driving the GARMI of [Robot Lobby](https://yxzhan.github.io/robot-lobby/),
a MuJoCo world in the browser, as if it were the real robot.

[![Binder](https://binder.intel4coro.de/badge_logo.svg)](https://binder.intel4coro.de/v2/gh/yxzhan/ijcai_2026_workshop/HEAD?urlpath=lab%2Ftree%2Fnotebooks)

## What is in here

- `notebooks/garmi_lobby_bridge.py`: the real GARMI's ROS interface on top of the lobby's ROS
  bridge (joint states, base odometry, the arms' velocity commands, franka_gripper's actions
  for the hands), so giskardpy's `garmi_velocity` and coraplex's `RealGarmiPlanExecutor` drive
  the lobby's GARMI unchanged.
- `notebooks/garmi_lobby_demo.py`: coraplex's GARMI demo (a bowl and a spoon carried across the
  apartment) on that executor.
- `binder/`: the image: ROS 2 Jazzy, the CRAM workspace (its venv and its ROS workspace
  `ros2_ws`), and in `ros2_ws` also `franka_msgs` and `topic_based_ros2_control`, with
  rosbridge.

## Running it

```bash
notebooks/run_garmi_demo.sh          # [--lobby r1] [--start -4.8 1.5 3.1416]
```

starts rosbridge (port 9090), waits for the lobby's GARMI to appear on ROS, starts the bridge
and giskard, and runs the demo once giskard is up; Ctrl-C stops them all, their logs are in
`~/.garmi_demo/`. In the lobby meanwhile: spawn GARMI alone, switch its IK off (Teleop panel)
and its ROS on (ROS panel, the rosbridge address).

By hand, the same in four terminals, each with the CRAM kernel's
environment (`~/.local/bin/cram_python_wrapper.sh` instead of `python`: ROS 2, `ros2_ws` and
the CRAM venv):

```bash
source /opt/ros/jazzy/setup.bash && ros2 run rosbridge_server rosbridge_websocket     # port 9090
~/.local/bin/cram_python_wrapper.sh notebooks/garmi_lobby_bridge.py
~/.local/bin/cram_python_wrapper.sh -m giskardpy.middleware.ros2.scripts.other_robots.garmi.garmi_velocity
~/.local/bin/cram_python_wrapper.sh notebooks/garmi_lobby_demo.py
```

Each script's docstring has the details (`--lobby r1`, `--start x y yaw`).

## Running it locally

```bash
git clone --recursive https://github.com/yxzhan/ijcai_2026_workshop.git
cd ijcai_2026_workshop
docker compose -f binder/docker-compose.yml up --build
```

Then open <http://localhost:8888>.
