#!/usr/bin/env bash
set -eo pipefail

export DISPLAY=:1.0
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/turtlebot3_ws/install/setup.bash
exec jupyter notebook --notebook-dir="/home/ubuntu/turtlebot3_ws/src/jupyter_notebooks" --ip 0.0.0.0 --no-browser
