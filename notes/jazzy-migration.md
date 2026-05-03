# ROS 2 Jazzy Migration Notes

This branch updates the workshop environment and notebooks for ROS 2 Jazzy on Ubuntu 24.04.

## Environment

- Docker now uses the Jazzy desktop VNC base image.
- The image installs Jazzy TurtleBot3, TurtleBot3 Gazebo, Nav2, Cartographer, ROS-GZ, and py_trees_ros packages.
- `docker-compose.yml` runs the published `adohaha/fun_ros:jazzy` image so workshop users do not need to build locally.
- Jupyter starts automatically inside the container after sourcing the ROS 2 Jazzy and workspace setup files.
- `.gitignore` excludes colcon build output, Python bytecode, Jupyter checkpoints, and Gazebo runtime state.

## Notebooks

- TurtleBot3 Gazebo `/cmd_vel` examples now use `geometry_msgs/msg/TwistStamped`, which is what the Jazzy TurtleBot3 Gazebo bridge expects.
- Nav2 examples launch with TurtleBot3 Jazzy parameters so Nav2 publishes stamped velocity commands.
- Nav2 initial pose setup uses QoS compatible with AMCL in Jazzy and publishes the pose repeatedly to avoid timing issues.
- Nav2 goal poses use zero timestamps where needed so TF resolves against the latest simulation time.
- Stale Humble notebook outputs and old branch setup instructions were removed.
- Unused old ROS distro image assets were removed.
- Service examples now use services available in the Jazzy ROS-GZ TurtleBot3 launch.
- The old Gazebo Classic reset-service helper path was removed because those services are not available in the Jazzy ROS-GZ TurtleBot3 launch.
- Behavior tree examples use the image-provided `py_trees` and `py_trees_ros` packages instead of installing `py_trees` from a notebook cell.

## Runtime State

- Generated Gazebo folders were removed from version control because they contained machine-local logs and old distro paths.
- Gazebo recreates those folders locally when the simulator runs.

## Runtime Checks

- Docker Compose starts `adohaha/fun_ros:jazzy` with Jupyter and noVNC running.
- TurtleBot3 Gazebo publishes `/scan`, `/odom`, `/tf`, camera topics, and subscribes to `/cmd_vel` as `geometry_msgs/msg/TwistStamped`.
- Publishing a `TwistStamped` command moves the robot in simulation.
- Nav2 starts with TurtleBot3 Jazzy parameters, accepts a `/navigate_to_pose` goal, and moves the robot.
