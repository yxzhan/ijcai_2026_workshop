#!/bin/bash

ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# The following line will allow the binderhub start Jupyterlab, should be at the end of the entrypoint.
exec "$@"
