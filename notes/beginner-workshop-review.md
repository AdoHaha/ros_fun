# Review: the beginner ROS workshop (exercises 1–5)

The first five exercises should make students want to explore robots and let them
explain what their code does. Behavior trees and planning belong to the separate
advanced course. This guide covers exercises 1–5; the root README describes the later course connections.
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
| NumPy 2 in the image conflicts with Ubuntu's Matplotlib binary | rqt Plot fails to open | NumPy compatibility repair is maintained with image tooling on the dev branch |
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
`exercises/.solutions/intro_answers.json`; copy its source into matching task cells
in an instructor notebook. Automated generation lives on the dev branch.

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

## Verification and image maintenance

The added automated tests, notebook runners, CLI/Gazebo probes, image recipes and
the detailed verification record live on the
[dev branch](https://github.com/AdoHaha/ros_fun/tree/dev). They supported the review;
students run the lesson notebooks and their visible task checks directly.

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
