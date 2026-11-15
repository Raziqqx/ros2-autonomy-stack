#!/bin/bash
# Source ROS and the built workspace, then run the given command.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
exec "$@"
