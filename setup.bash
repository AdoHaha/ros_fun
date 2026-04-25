#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

cd /home/ubuntu/turtlebot3_ws
colcon build --symlink-install

grep -qxF 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || \
  echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
grep -qxF 'source ~/turtlebot3_ws/install/setup.bash' ~/.bashrc || \
  echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
grep -qxF 'export TURTLEBOT3_MODEL=waffle_pi' ~/.bashrc || \
  echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
grep -qxF 'export DISPLAY=:1.0' ~/.bashrc || \
  echo 'export DISPLAY=:1.0' >> ~/.bashrc
