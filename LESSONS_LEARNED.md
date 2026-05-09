# Lessons Learned

Notes for future work in this repository and its Docker ROS/Jupyter environment.

## Docker and ROS Environment

- The working container is named `ros_fun`.
- The current Docker image is `adohaha/fun_ros:jazzy`.
- Verify the ROS version inside the running container, not from host assumptions:

  ```bash
  docker exec --user ubuntu ros_fun bash -lc 'echo $ROS_DISTRO && printenv AMENT_PREFIX_PATH | tr ":" "\n" | head'
  ```

- At the time of this work the container reports ROS Jazzy, not Humble.
- The workspace inside the container is:

  ```text
  /home/ubuntu/turtlebot3_ws
  ```

- The host `jupyter_notebooks/` directory is mounted into the container at:

  ```text
  /home/ubuntu/turtlebot3_ws/src/jupyter_notebooks
  ```

## Build Commands

Build the notebook package from inside the container:

```bash
docker exec --user ubuntu ros_fun bash -lc \
  'source /opt/ros/jazzy/setup.bash && cd /home/ubuntu/turtlebot3_ws && colcon build --symlink-install --packages-select ros_fun'
```

For shell commands that use built packages, source both ROS and the workspace overlay:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/turtlebot3_ws/install/setup.bash
```

## Python Package Details

- In this Jazzy container, `py_trees` and `py_trees_ros` are installed from ROS packages and report version `2.4.0` via Python package metadata.
- Do not use `py_trees.__version__`; this attribute is not available here.
- Use:

  ```python
  from importlib.metadata import version

  print(version("py_trees"))
  print(version("py_trees_ros"))
  ```

## py_trees_ros Tutorial Notes

- The upstream package name `py_trees_ros_tutorials` can conflict with an installed upstream package. The vendored copy in this repo is named `ros_fun_py_trees_ros_tutorials`.
- Notebook launch examples should use the local package name `ros_fun`.
- Qt dashboard nodes are not reliable for headless notebook exercises. Prefer driving the same behavior through ROS topics such as:

  ```bash
  ros2 topic pub --once /dashboard/scan std_msgs/msg/Bool '{data: true}'
  ros2 topic pub --once /dashboard/cancel std_msgs/msg/Bool '{data: true}'
  ```

## Introspection Watchers

- `py-trees-tree-watcher -b` can fail if no snapshot stream is open.
- For the tutorial tree node `/tree`, enable snapshot parameters before starting the watcher:

  ```bash
  ros2 param set /tree default_snapshot_stream True
  ros2 param set /tree default_snapshot_blackboard_data True
  ros2 param set /tree default_snapshot_blackboard_activity True
  py-trees-tree-watcher -a -s -b /tree/snapshots
  ```

## Validation Practice

- Validate notebooks cell by cell inside Docker, not just with static inspection.
- Treat background terminal commands as part of validation: if a launched background process exits nonzero immediately, the notebook cell should be considered failed.
- Before repeated ROS launch tests, clean up stale processes with a bracketed regex so the cleanup command does not match itself:

  ```bash
  pkill -f '[r]os2 launch ros_fun|[p]y-trees-tree-watcher|[p]y-trees-blackboard-watcher|[t]ree-action-clients|[m]ock-battery|[m]ock-led-strip|[m]ock-docking-controller|[m]ock-move-base|[m]ock-rotation-controller|[m]ock-safety-sensors' 2>/dev/null || true
  ```
