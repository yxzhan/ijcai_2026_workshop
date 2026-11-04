#!/bin/bash

# rosbridge (port 9090), for the lobby's ROS panel: in the background, its output in
# ~/.rosbridge.log. ROS sourced here (the entrypoint has no login shell), and on the
# system's python3: rosbridge's scripts run `#!/usr/bin/env python3`, which would be
# conda's, without rosbridge's Python dependencies (bson, cbor2, ...: apt's, for
# /usr/bin/python3).
(
    source /opt/ros/jazzy/setup.bash
    PATH=/usr/bin:${PATH} exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml
) > "${HOME}/.rosbridge.log" 2>&1 &

# The following line will allow the binderhub start Jupyterlab, should be at the end of the entrypoint.
exec "$@"
