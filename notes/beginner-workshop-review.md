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
```

The notebook runner uses fresh kernels and a separate ROS domain (87), starts real
GUI turtlesim processes, and writes executed notebooks/reports to
`/tmp/intro-notebook-verification`. It excludes manual CLI/teleop and optional Gazebo
cells. It builds the actual widgets, clicks Forward through their real handlers,
checks observed motion and stop, checks all three checkpoint scores with pose data,
and requires false → true → false delivery results after actual motion. `--repeat 2`
checks a full rerun in the same kernel. The ten helper regressions cover variable
scan geometry/invalid readings, spin-dependent timers, real services, unavailable
and unresponsive servers, stopping on callback failure, and stamped commands.

Manual classroom checks still matter: focus the teleop terminal, draw a recognizable
letter, pair up for the radio challenge, and try the optional Gazebo/camera/laser
extensions. Automated execution cannot establish that students find an activity fun.
Try the sequence with a small beginner group and observe where they ask for help.

Primary references: [Jazzy topics](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html),
[Jazzy services](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html),
[Jazzy QoS](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html).
Interface definitions and execution were also checked against the installed Jazzy
packages in `ros_fun`.
