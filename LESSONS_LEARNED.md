# Lessons Learned

Notes for future work in this repository and its Docker ROS/Jupyter environment.

## Course connections and continuation (2026-10-09)

- Exercises 6–8 continue the courier story. Six is an optional real Nav2 route;
  seven teaches the action lifecycle with standalone turtlesim; eight validates
  parameters and demonstrates their effect on actual speed and arrival rules.
  Neither seven nor eight depends on a running navigation world.
- The advanced course retains its tree and planning tasks. Entry explanations
  connect observations, configuration, running actions and cancellation to tree
  priorities, then distinguish Nav2 route planning from symbolic task planning.
  Planning notebooks 12–14 now have Polish descriptions consistent with the rest.
- Optional exercise 15 supplies a controlled, headless execution bridge. Effects
  apply only on terminal success; failure, cancellation and changed observations
  cannot silently commit the old action. One still-legal local retry is separate
  from replanning. This is preparation for a restaurant ROS adapter, not one.
- Navigation goals need finite coordinates, simulation timestamps and unit yaw
  quaternions. Goal acceptance and cancel acknowledgement are not terminal results.
  Startup and result waits have wall deadlines and useful log paths.
- Publish the initial AMCL pose at a bounded rate while continuing to process
  callbacks. Publishing after every spin floods localisation with resets when
  other subscriptions return immediately. A regression exercises that scenario.
- Navigation notebooks 6 and 11 own their Gazebo/Nav2/RViz process groups and
  refuse another running world. Cleanup stops only resources that the notebook
  created; it continues signalling owned children if the launcher exits first.
  The installed simulation is turtlebot3_world, despite legacy Castle code names.
- Jazzy RotateAbsolute returns delta toward the starting heading: a positive
  quarter turn yields approximately negative pi/2. Check actual heading and
  terminal status rather than inferring success from the result sign.
- Parameter callbacks validate type, finiteness and range before changing values.
  Atomic batches reject all updates if one is invalid; ordinary batches may
  succeed partially. Fresh CLI discovery avoids stale cached notebook nodes.
- Nav2 arrival verification uses the current map-to-base TF transform: AMCL's
  last published pose may be older than the final robot position. Cancellation
  checks require a fresh stopped odometry sample and the terminal canceled result.
- The combined exercise regression suite passed 117 tests. Live notebook eleven
  completed patrol, goal preemption, successful arrival, resumed patrol and stop
  with RViz, recording 1.248 m of travel and a final zero velocity command.
- Exercise 6 passed two starter and two solved runs in the same kernel, including
  the actual exercise-7 native Nav2 action cell on both solved runs. Its domain
  was empty after cleanup. Final orientation can take time even when the remaining
  distance is small; the lesson explains why the terminal result is still needed.
- Public navigation and parameter-CLI timeouts reject nonfinite values rather
  than allowing NaN or infinity to bypass wall deadlines.
- The restaurant model/executor suite passed 21 tests. Student demonstrations
  12–14 and all six student/reference tree notebooks passed kernel execution.
  Exercise 15's student and solved copies each passed two runs in one kernel;
  exercises 7–8 did likewise with real turtlesim, feedback, cancellation,
  parameter validation and CLI requests.


## Beginner workshop review (2026-10-08)

- Exercises 1–5 now follow a courier story: sketching and graph discovery, radio
  messages and timed publication, position-based checkpoint scoring, then a
  delivery service. Student-facing explanations remain Polish; advanced notebooks
  are unchanged. See [the review and teaching guide](notes/beginner-workshop-review.md).
- Use turtlesim for the beginner path to avoid making slow Gazebo startup a
  prerequisite. TurtleBot/Gazebo/camera and LaserScan remain optional extensions.
- Keep typed publishers, subscriptions, timer callbacks, client/future handling
  and the service callback visible. Move repeat-run housekeeping into a helper;
  label the setup cell as ready to run.
- Finite spinning makes callback behavior observable without blocking the kernel
  indefinitely or creating multiple background executor threads. Distinguish a
  message being published from the robot receiving or acting on it.
- Remove ipywidget click handlers with `on_click(..., remove=True)`;
  `unobserve_all()` does not remove those handlers.
- Sensor queues need compatible QoS. LaserScan geometry comes from the message's
  actual angles and limits; invalid-only forward readings mean unknown space.
- A discovered service may never reply. Bound discovery and response waiting,
  remove pending requests on timeout, and explain that timeout does not undo the
  server's operation. A negative Trigger response can be valid game feedback.
- Verification uses real ROS and turtlesim in a separate ROS domain, triggers the
  actual button handlers, measures movement and stopping, checks one score per
  checkpoint, and confirms false → true → duplicate-refused delivery responses.
  Student notebooks run with unfinished tasks; instructor answers are injected
  only into generated solved copies. Full solved runs also repeat in one kernel.
- The exercise regression suite passed 92 tests, including 10 sensor/lifecycle
  and 13 task/boundary/ownership regressions. The verifier checks every control
  button, all square corners and closure, and checkpoint collection by driving.
- `verify_intro_cli.py` passed 24 live keyboard/CLI/radio/service checks.
  `verify_intro_gazebo.py --gui` checked the real world, camera, laser notebook
  cell, simulation-stamped movement, odometry and stop. RViz camera, Plot curve
  and Node Graph were inspected on the desktop. Classroom pair networking and
  beginner enjoyment still require testing with students.
- Jazzy turtlesim advances a fixed 16 ms physics step per Qt timer update. Under
  heavy Gazebo/GUI load, a one-second wall-time command can produce a smaller
  turn than expected. Close the optional Gazebo/RViz/rqt before core turtle
  missions; preserve this honest open-loop limitation rather than hiding pose
  feedback inside the publisher lesson. The subscriber lesson introduces pose.
- Allow several seconds for DDS discovery before deciding an existing turtlesim
  is absent; one second started duplicate simulators under load. A fresh-context
  regression checks reuse and ensures a borrower cannot close the owner's robot.
- NumPy 2 in the prebuilt image broke Ubuntu's Matplotlib extension and rqt Plot.
  Both Docker recipes pin NumPy 1.26.4 and check Matplotlib/OpenCV imports. The
  running container was repaired for validation; Docker Hub still needs a new
  image release before ordinary Compose users receive that fix.

## Behavior tree workshop verification (2026-10-08)

- Exercises 9–11 now teach decisions across ticks: reactive guards, workflow
  memory, parallel completion, finite recovery, action cancellation and ownership.
  Students edit and execute notebook cells, including demos, diagrams and cleanup.
- Reference notebooks in `exercises/.solutions/` contain executable solutions.
  The deterministic checks also reject the deliberately incorrect student
  starters and memory mutations; a printed tree alone is not verification.
- `py_trees` DOT/SVG diagrams come from the actual tree objects. Generated files
  are placed under `/tmp/ros_fun_bt_diagram_*`, outside notebook sources.
- Use `exercises/verify_bt_notebooks.py --ros` for fresh-kernel notebook runs.
  It executes mock ROS only for the solved application; unfinished student ROS
  cells require their task checks to pass before launching anything.
- `BasicNavigator` must have `use_sim_time=True` before stamping poses in Gazebo.
  Its `cancelTask()` waits for acknowledgement, not the terminal action result.
  The demo polls completion before allowing another leaf to reuse the navigator.
- A reactive selector ticks the new higher priority branch before invalidating
  the old lower priority branch. Explicit action ownership prevents the old
  leaf's cleanup from canceling the new goal. A single LED output publisher
  emits the final selected color after invalidation finishes.
- `ScanContext` now uses asynchronous get/set/restore callbacks. Tutorial six
  gates rotation on confirmed context readiness. The executor must keep spinning
  for restoration callbacks after branch invalidation.
- The mock ROS solution was executed end to end: action success, server-confirmed
  cancellation, battery interruption and resumed completion. Tutorial six was
  also checked against real parameter services and a real mock action server.
- On this host Gazebo ran around 0.05 of real time. Notebook eleven measures
  `/clock` progress to make this visible. The default goal faces the approach
  direction to avoid a long final rotation; the existing castle world is used.
- Live notebook eleven completed patrol → requested goal → successful result →
  resumed patrol → stop. The final motion probe recorded 1.21 m, 715 nonzero velocity
  samples and a final zero command. Waiting for terminal cancellation eliminated
  the immediate goal failure observed during the earlier priority handoff.
- The live status view uses an HTML widget rather than repeatedly clearing an
  Output widget from a background thread; the latter broke nbclient output capture.
- Beginner-facing tasks include definitions, expected traces, optional hints and
  separate recovery/resource checks. Notebook eleven is explicitly an extension.
- All six student/reference notebooks passed fresh-kernel execution, including
  the mock ROS solution. The 69 regression tests cover the task checks, ROS
  adapter, asynchronous context, navigation ownership/readiness and notebook
  command timeouts (including termination of child pipelines).
- Nav2 startup, parameter/costmap services and action acknowledgements have wall
  deadlines. Missing idempotent service replies are retried; a goal accepted
  after a send timeout is canceled rather than left without an owner.
- The Nav2 notebook checks readiness using one persistent ROS client, rather
  than a pipeline of fresh CLI processes whose discovery may time out.
- The kitchen world and planning exercises are separate work.

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
- The mock robot launch starts the Qt dashboard. In the VNC desktop it should appear as a separate window when a tutorial launch file is started from a notebook terminal.
- The same dashboard interactions can also be driven from notebook cells with ROS topics:

  ```bash
  ros2 topic pub --once /dashboard/scan std_msgs/msg/Empty '{}'
  ros2 topic pub --once /dashboard/cancel std_msgs/msg/Empty '{}'
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
- Before repeated ROS launch tests, use the notebook's cleanup cell or Ctrl+C in
  the terminal that owns the launch. Use an isolated ROS domain for verification.
  Avoid broad process-name cleanup that could stop another student's session.
