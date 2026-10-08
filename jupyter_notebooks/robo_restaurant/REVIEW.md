# Robo-restaurant implementation review

Reviewed and exercised on 2026-10-08 using the local `adohaha/fun_ros:jazzy`
image, ROS 2 Jazzy, Python 3.12 and Gazebo Sim 8.11.0. This is a self-review,
including runtime checks, rather than an independent reviewer sign-off.

## Results

- **11 automated tests pass.** Coverage includes 18 initial battery/device
  configurations, every transition reachable from the default state, action
  preconditions, single-tray capacity, monotonic order progression, battery
  bounds, known optimal costs, stale-action rejection and unreachable cases.
- **Nine notebook executions pass.** Three untouched student starters execute
  their demos and explicitly mark the answer unfinished. Three student notebooks
  filled with reference answers pass their checks. Three separate instructor
  notebooks pass their checks and their enabled Gazebo integration cells.
- **The SDF validator accepts the world.** A running headless world loads all
  21 expected models and publishes simulation statistics.
- **All three solutions pass live Gazebo pose verification.** Normal service
  completes 13 actions at cost 25; low battery completes 14 actions at cost 28;
  cooker failure completes 14 actions at cost 30, including repair. Every action
  and the injected failure update are checked against the live pose topic.
- **Service pose clearance passes** for a circular waiter of radius 0.35 m
  against the static obstacle boxes. This checks station clearance rather than
  physical navigation.
- **GUI visual inspection performed.** The room, kitchen, tables, customers,
  dock, service markers and waiter appear in the scene. A red marker appears
  above the failed cooker. The visible-world recovery run ends with meals on
  both tables, the fault cleared and the waiter at the dock.

Executed notebooks are retained in `/tmp/robo-restaurant-validation/notebooks`
on the review host; source notebooks contain no saved execution outputs.

## Findings fixed during review

1. The original symbolic demos had no Gazebo effects. Added a discrete world
   adapter and waiter/meal/fault proxies, then verified each solution transition.
2. Gazebo's `scene/info` reports initial model poses even after pose updates.
   Verification initially failed when the waiter moved. Switched verification
   to `/world/robo_restaurant/pose/info`; added a parser regression test.
3. A default camera cropped parts of the scene. Added an explicit overview
   camera and a small GUI configuration for inspecting the entire layout.
4. Notebook scaffolds must not appear to pass while unanswered. Unimplemented
   cells now print TODO; the test runner separately verifies filled exercises.
5. Pose changes are sent in one blocking vector request, then verified from
   live feedback, reducing intermediate visual inconsistencies and CLI overhead.

## Scope and remaining work

No unresolved defect was observed in the tested symbolic/discrete-visual scope.
The proxy teleports between stations and has no collisions. The planner owns
restaurant state; Gazebo provides verified visualization, not independent cook,
customer or battery dynamics. There is no Nav2 waiter, live ROS restaurant state
publisher, asynchronous device operation or scikit-decide adapter yet. Those
require a subsequent integration milestone; these tests make no claim about
collision-free driving or sensor-based execution.

This `ag2` instructor branch retains `solutions/` and the full verification
tools/tests. Student `master` omits those files and lists the notebooks as
exercises 12–14. The earlier local commit contains answers in Git history;
branch separation controls normal browsing, not historical access.

## Reproduce

From the repository root with the workshop image available:

```bash
docker run --rm --entrypoint bash \
  -v "$PWD/jupyter_notebooks:/lab:ro" -w /lab \
  adohaha/fun_ros:jazzy -c '
    source /opt/ros/jazzy/setup.bash
    export ROBO_RESTAURANT_GAZEBO=1
    /usr/bin/python3 -m unittest discover -s robo_restaurant/tests -v &&
    /usr/bin/python3 -u robo_restaurant/tools/verify_notebooks.py --output /tmp/restaurant-notebooks &&
    /usr/bin/python3 -u robo_restaurant/tools/verify_gazebo.py
  '
```

The headless integration checks create private transport partitions and stop
their own servers. The optional `live` notebook mode deliberately modifies the
already-open restaurant world so an instructor can inspect the execution.
