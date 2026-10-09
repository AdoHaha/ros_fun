# Review: the beginner ROS workshop (exercises 1–5)

The first five exercises should make students want to explore robots and let them
explain what their code does. Behavior trees and planning belong to the separate
advanced course. Exercises 6–14 and their runtime helpers are outside this change.
Student text remains Polish, with familiar ROS interface names in English.

## Findings and changes

| Original issue | Effect on students | Change |
| --- | --- | --- |
| Definitions and commands without predictions, outcomes or milestones | Hard to connect code with robot behavior | One courier story, explicit objectives, predict/run/change experiments, expected output and short reflections |
| Heavy Gazebo startup was the prerequisite for almost everything | Slow machines spend the beginner lesson waiting | Fast turtlesim core; keep TurtleBot/Gazebo/camera and laser extensions |
| Exercises 2 and 3 both launch Gazebo without ownership checks | Duplicate simulation, confusing topic graph and competing commands | Each notebook discovers/reuses turtlesim; only stops its own launch; Gazebo startup is explicitly optional |
| Raw `publisher.publish("hello")` intentionally raises an unhandled error | Run-all fails before the working example | Catch and explain the expected type error next to the correct typed message |
| Timers and spin shown with little explanation; blocking waits without deadlines | Kernel appears stuck; students confuse sending with receipt | Explicit callback/executor/future model, finite spin experiments and bounded discovery/reply waits |
| Only one movement button connected, STOP shown twice | A control panel that mostly does nothing | Every button has a real callback, finite movement and a final zero command |
| `unobserve_all()` used to remove button handlers | Click callbacks remain registered | Remove click handlers with `on_click(..., remove=True)` and close old widgets |
| Subscriber task reads an undefined variable; empty placeholders fail | Students cannot tell a starter from a bug | Runnable starter functions, visible feedback, hints and gates before scoring or delivery |
| Copied spinner can start multiple unjoined, non-daemon threads | Executors race; cleanup and kernel shutdown are unreliable | No background spinner in the beginner path; one executor owned by each notebook |
| Laser plot assumes 360 values and first reading means forward | Wrong shape or wrong obstacle direction | Optional laser uses message angles, range limits and sensor-data QoS; invalid-only cone stays unknown |
| rqt Plot asks for a structured position object | No useful numeric plot | Use `/odom/pose/pose/position/x` |
| NumPy 2 in the image conflicts with Ubuntu's Matplotlib binary | rqt Plot fails to open | Pin NumPy 1.26.4 in both image recipes and check Matplotlib/OpenCV imports during build |
| A fresh ROS context can take longer than one second to discover turtlesim | A second simulator starts despite an existing session | Allow five seconds for discovery and test reuse from a separate context |
| Services only inspect another node's parameters | Little visible payoff; no server-side model | Change pen, draw a postcard, teleport, then implement a Trigger delivery server with positive, negative and duplicate responses |
| Service call waits indefinitely for reply | Discovery success can still leave the notebook stuck | Bounded waits for both discovery and completion; clients and pending requests cleaned up |

## Teaching sequence

1. **Introduction (10 min):** verify container/kernel, distinguish host and container,
   name the communicating nodes and choose topic/service/action for three examples.
2. **Drive (15–20 min):** robot signature, graph detective, predict a velocity command.
   TurtleBot parking and camera are optional teacher demonstrations.
3. **Publisher (25–35 min):** radio password, typed messages, timer versus sleep,
   numbered messages, square drawing and working control buttons.
4. **Subscriber (25–35 min):** latest pose, callback count, distance challenge,
   three checkpoints with a map and one point per checkpoint. Laser is an extension.
5. **Service (25–35 min):** inspect Request/Response, call SetPen explicitly,
   three-color postcard, teleport, implement delivery eligibility, drive to delivery,
   and reject duplicate collection. Optional terminal client reinforces server spin.

For a short session, do the basic missions and omit every bonus. Students should
make small edits in the notebook, not copy an entire robot package. Completed
student starters are intentionally not prefilled. Instructor answer cells are in
`exercises/.solutions/intro_answers.json`; the runner produces solved notebooks
outside the checkout. The existing behavior-tree solutions remain unchanged.

## Runtime design

`intro_ros.py` contains setup, bounded motion/wait/cleanup utilities and optional laser geometry.
Publishers, subscriptions, timers, client/future handling and service callbacks stay
visible in the notebooks. Each lab has its own rclpy Context and a single-threaded
executor; no hidden background processing. Setup handles a full notebook rerun,
including old widget callbacks. That housekeeping lives in `notebook_lab`, so the
first cell stays short and is explicitly labeled as ready-to-run setup. Motion sends at 10 Hz and always attempts a final
zero command. The lab closes only the simulator process group it created. An
existing shared simulator is reused with a message asking students to stop other
controllers. ROS discovery and simulator startup take time and have deadlines.

Turtlesim uses `Twist`; this container's TurtleBot Gazebo uses `TwistStamped`.
The notebook explains that difference and asks students to inspect the actual type.
The laser extension explains how to enable `use_sim_time` and wait for `/clock`
before stamping Gazebo commands. TurtleBot keyboard control must be stopped with
`s` before Ctrl+C: the installed teleop publishes its current target speed again
during shutdown, so Ctrl+C alone is not a reliable stop command. End the teleop
process after that explicit stop so it cannot command a later simulator session.

Close the optional Gazebo, RViz and rqt before the turtle missions. Jazzy turtlesim
integrates a fixed 16 ms step for each Qt timer update, so delayed updates under
heavy load shorten movements controlled only by wall time. A concurrent stress
run with all visual tools open failed the square's geometric check. After closing
the visual tools, measured sides were 0.992–1.008 units and turns were 1.533–1.583
radians. A controlled pause of the owned simulator confirmed that this is real
movement skew, rather than a pose-probe error. The notebook explains the limitation
and the next lesson introduces position feedback. See the
[Jazzy turtlesim timer implementation](https://github.com/ros/ros_tutorials/blob/jazzy/turtlesim/src/turtle_frame.cpp).

QoS queue depth is distinguished from publish frequency. Volatile topics do not
replay history to a late subscriber. Service responses are distinct from application
success; timeout does not undo an operation already performed by the server.

## Language review

The final Polish copy pass covered instructions, hints, reflections, code comments
and game feedback in exercises 1–5. It corrected grammar and punctuation, removed
awkward English/Polish combinations, introduced “węzeł (node)” consistently, and
clarified angular velocity, kernel restart, callback registration and simulator
reuse. It also checked that each instruction describes the implemented behavior.
Only explanatory text and feedback changed during this pass; ROS interface names,
message fields, task logic and advanced exercises retain their existing behavior.

## Maintainer verification

Inside the workshop container (the source directory is bind-mounted):

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ubuntu/turtlebot3_ws/src/jupyter_notebooks/exercises
python3 verify_intro_notebooks.py
python3 verify_intro_notebooks.py --solutions --repeat 2
ROS_DOMAIN_ID=88 python3 -m unittest discover -p 'test_*.py'
python3 verify_intro_cli.py
python3 verify_intro_gazebo.py --gui
```

The notebook runner uses fresh kernels and a separate ROS domain (87), starts real
GUI turtlesim processes, and writes executed notebooks/reports to
`/tmp/intro-notebook-verification`. It excludes manual CLI/teleop and optional Gazebo
cells. It clicks every widget button through its real handler, checks signed movement
and final stop, verifies all four corners and closure of the square, drives to all
three checkpoints without teleporting, and requires false → true → false delivery
results after actual motion. `--repeat 2` checks a full rerun in the same kernel.
The ten helper regressions cover variable
scan geometry/invalid readings, spin-dependent timers, real services, unavailable
and unresponsive servers, stopping on callback failure, and stamped commands.
Thirteen additional solvability regressions check the instructor functions,
two-dimensional delivery geometry and boundary conditions, unknown/stale positions,
unfinished-task gating, receipt uniqueness, and simulator reuse/ownership. Together
with the existing advanced-course regressions, all 92 tests passed in the container.

The CLI verifier passed 24 checks using real controlling terminals for all four
turtlesim arrows and TurtleBot keyboard
input, verifies documented graph/introspection commands, observes a published arc,
and checks an external radio subscriber, Trigger service client and GetParameters
request. It writes a report to `/tmp/intro-cli-verification`. The Gazebo verifier
launches the actual world, receives a nonblank 640×480 camera image and 360-sample
scan, runs the notebook's optional laser cell, drives with simulation-stamped
TwistStamped messages, and checks odometry and stop. `--gui` also opens RViz, Plot
and Node Graph and saves a desktop screenshot. GUI screenshots were inspected:
RViz received the camera image, Plot traced the actual movement followed by a flat
line after stop, and the graph rendered the real nodes. Reports and
images go to `/tmp/intro-gazebo-verification`. Both verifiers use separate domains,
bounded waits, and stop only their own launched process groups. Do not use their
domains for a simultaneous student session.

Fresh Gazebo verification also checks that its ROS domain is unused before launch.
A collision probe correctly refused to launch or publish into an active simulation;
the original verifier completed normally. Cleanup still stops owned processes if
ROS lab shutdown raises, and CLI cleanup was checked with a stubborn child process
whose launcher had already exited.

The NumPy fix was applied and tested in the running container. A refreshed Docker
image must be built and published before ordinary `docker compose up` users receive
it; editing a Dockerfile does not update the existing Docker Hub image. The image
recipes now check imports during build; a complete image rebuild is a separate
release step. See [NumPy's ABI troubleshooting guidance](https://numpy.org/doc/stable/user/troubleshooting-importerror.html).

Manual classroom checks still matter: draw a recognizable letter, pair up on the
classroom setup for the radio challenge, and observe how beginners use the controls.
The radio test uses separate processes in one container; communication between
students' machines still depends on their networking. Automated execution cannot
establish that students find an activity fun.
Try the sequence with a small beginner group and observe where they ask for help.

Primary references: [Jazzy topics](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html),
[Jazzy services](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html),
[Jazzy QoS](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html).
Interface definitions and execution were also checked against the installed Jazzy
packages in `ros_fun`.
