#!/bin/bash
set -e

# source ROS 2 environment
source "/opt/ros/humble/setup.bash"
exec "$@"