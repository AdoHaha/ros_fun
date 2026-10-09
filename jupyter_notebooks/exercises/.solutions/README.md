Executable reference solutions for the behavior tree exercises. Open these
notebooks from `.solutions/`; their setup also works from the student folder.
The student notebooks have deliberately incomplete tree builders. Their commented
checks should pass after students solve the tasks. Each solution replaces those
builders and runs the checks, rather than describing a patch students cannot run.

Suggested teaching order:

1. **9**: run the deterministic demo, compare selector memory, then solve reactive
   battery guards, parallel notification and one-shot event memory.
2. **10**: build bounded recovery, resource scope and reactive priorities. Run the
   tick scenarios, then connect the same tree to the mock ROS robot. Its final
   cells verify action success, server-confirmed cancellation, battery interruption
   and restart, and stop the mock launch they created.
3. **11**: transfer the same ideas to the TurtleBot/Gazebo Nav2 demo.

Students work entirely in notebook cells: edit the tree builders, run checks,
generate diagrams, start demos and clean up. The commands below are for maintainers.
DOT/SVG diagrams are generated from the students' current trees, including their
memory settings; the full application diagram and focused scan diagram support
discussion of priority, recovery and parallel execution.

Exercise 10 keeps the sensor scope and repair step simulated. Its action clients,
battery input, dashboard topics and LED output use real ROS. The vendored py_trees_ros tutorial six separately
demonstrates asynchronous sensor parameter services. The kitchen world and planning
tasks are separate work; these exercises do not require them.

Run verification inside the workshop container, from `exercises/`, with ROS sourced:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/turtlebot3_ws/install/setup.bash
cd /home/ubuntu/turtlebot3_ws/src/jupyter_notebooks/exercises
python3 verify_bt_notebooks.py --ros
python3 -m unittest discover -p 'test_*.py' -v
python3 bt_demo.py
python3 bt_demo.py --memory
```

To verify the complete optional Nav2 notebook, including real simulated motion:

```bash
python3 verify_bt_nav2.py
```

This launches the TurtleBot world in Gazebo through the notebook with owned
Gazebo/Nav2/RViz process groups, checks patrol,
requested goal completion, resumed patrol and stop, then runs notebook cleanup.
It saves the executed notebook and motion/event reports in
`/tmp/bt-nav2-verification`. Allow several minutes on hosts with slow simulation.

The notebook runner uses a fresh kernel for each notebook and writes executed
copies and `report.json` to `/tmp/bt-notebook-verification`. It runs the student
demonstrations without requiring unfinished tasks to pass. Real ROS cells only run
in the solution notebook. Stop any other mock robot tutorial before this pass;
multiple trees or mock servers must not share `/rotate` and the LED strip.

The original exercises had an unreachable warning after a permanently `RUNNING`
leaf, mostly cosmetic ROS tasks, competing LED publishers in a proposed solution,
and cancellation advice that could command movement without an active request.
The replacement tasks check transitions across ticks, action ownership and cleanup,
finite completion, limited recovery and input handling. Merely printing a tree is
insufficient to pass those checks.

## Beginner exercises 1–5

`intro_answers.json` contains replacements for the four marked task cells: numbered
messages, square velocities, checkpoint distance and delivery eligibility. The
student notebooks stay runnable with unfinished tasks and explain what to change.
Generate and execute solved copies inside the sourced workshop container:

```bash
cd /home/ubuntu/turtlebot3_ws/src/jupyter_notebooks/exercises
python3 verify_intro_notebooks.py --solutions --repeat 2
```

The runner writes solved notebooks to `/tmp/intro-notebook-verification`, constructs
and clicks the real control buttons, validates checkpoint scoring, and checks a
real drive followed by successful and duplicate-refused delivery requests. See
[the beginner teaching guide](../../../notes/beginner-workshop-review.md) for pacing
and review findings.

## Continuation and planning bridge

Exercises 6–8 have separate JSON answers: `navigation_answers.json` uses
`course_task`, `action_answers.json` uses `action_task`, and
`parameters_answers.json` uses `parameters_task`. The starter notebooks run with
unfinished tasks and report what remains to be completed. Use their verifiers
inside the sourced container:

```bash
python3 verify_navigation_notebook.py --solutions --repeat 2
python3 verify_navigation_notebook.py --solutions --action-bridge
python3 verify_action_notebook.py --solutions --repeat 2
python3 verify_parameters_notebook.py --solutions --repeat 2
```

Omit `--solutions` to test the starter. These runners use separate ROS domains
(95, 96 and 97 respectively). Run Gazebo verification sequentially and close other
heavy GUI demonstrations before the turtle checks. Executed notebooks and reports
go under `/tmp`. Nav2 verification checks current TF position, measured motion,
terminal cancellation and fresh stopped odometry; acceptance alone is insufficient.
The `--action-bridge` option also runs exercise 7's optional native Nav2 client
against the owned world, checking its final status and current map position.

Exercise 15's `planning_execution_answers.json` replaces the cell marked
`planning_execution_task='recovery_policy'`. From `jupyter_notebooks` run:

```bash
python3 robo_restaurant/tools/verify_execution_bridge.py --repeat 2
python3 robo_restaurant/tools/verify_execution_bridge.py --solutions --repeat 2
```

This is a headless controlled executor, not a restaurant Nav2 integration. It
checks successful effects, failed-action rollback, bounded local retry, observed
state changes, replanning and deliberate cancellation. Exercises 12–14 keep their
existing planning tasks; the new Polish descriptions connect them to execution.
