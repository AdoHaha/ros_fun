# Fun with ROS 2

Robotics is fun and ROS 2 is a must.

In this workshop, we will explore ROS 2 by playing with simulated robots.

You need to have Docker and docker compose installed.
It can be docker engine:
Instructions for Ubuntu are provided [here](https://docs.docker.com/engine/install/ubuntu/)

but for beginners, an easier option might be Docker Desktop: [Windows](https://docs.docker.com/desktop/install/windows-install/) [Linux](https://docs.docker.com/desktop/install/linux-install/), [Mac](https://docs.docker.com/desktop/install/mac-install/)




![Workshop setup: clone master, start Docker Compose, and open the desktop and notebooks](docs/images/workshop-setup.gif)


 I suggest cloning this repository (you need to install [git first](https://github.com/git-guides/install-git)).


`git clone --branch master https://github.com/AdoHaha/ros_fun.git`

`cd ros_fun`

Use 

`docker compose up` to pull and run the ROS 2 Jazzy container.

Access the virtual machine screen by navigating to 

[http://localhost:6080](http://localhost:6080)

access the jupyter notebooks by navigating to:

[http://localhost:8888](http://localhost:8888) 

on your **host** machine. 

Open [exercise 1](http://localhost:8888/notebooks/exercises/1.%20introduction.ipynb)
in Jupyter to start the workshop.

The demo uses ROS 2 Jazzy on Ubuntu 24.04.

## Updating an existing workshop installation

Save your work in Jupyter and back up edited notebooks before updating the
repository. Finish any running robot commands, then run these commands in the
repository folder on your computer:

```bash
git pull --ff-only
docker compose pull ros2
docker compose up -d ros2
```

The image repair fixes NumPy compatibility with the plotting and camera tools.
With the previous Hub image cached, the added download is about 22 MB on
Intel/AMD or 18 MB on ARM. Older cached releases may also need earlier base
updates. Pulling does not
update a running container; `up -d` recreates it when its image changes. Your
notebooks are stored in the repository's mounted folder and persist across
container recreation.

## Beginner workshop: exercises 1–5

The Polish notebooks start with a robot courier game in **turtlesim**: drive and
sketch, send radio messages, build a control panel, collect position-based points,
and implement a service that confirms a delivery. Each mission includes explanations,
small challenges, expected results, hints and cleanup. TurtleBot/Gazebo and laser
activities are optional extensions so slow simulation does not block the core workshop.

Start with [exercise 1](jupyter_notebooks/exercises/1.%20introduction.ipynb).
The [review and teaching guide](notes/beginner-workshop-review.md) describes the
changes, suggested pacing and verification commands.

## From driving to autonomous decisions

The student descriptions in exercises 1–15 are Polish. Exercises **6–8** continue
the courier story: an optional Nav2 delivery route, a standalone turtlesim compass
challenge using actions, and configurable delivery rules using parameters. Nav2
is optional; exercises 7–8 do not require Gazebo or exercise 6's setup.

Exercises **9–15** belong to the separate advanced course and include a short
ROS recap. The connections are explicit: subscribers supply observations,
parameters configure rules, and actions remain running until their result arrives.
Behavior trees choose priorities and handle interruption; planners select an
action sequence from the observed state. The final optional bridge practices
execution, bounded retry and replanning before adding a restaurant ROS adapter.

## Behavior tree lab

Behavior tree exercises start with `exercises/9. Behavior Trees.ipynb`: a small
deterministic demo and tasks about reactivity, memory and parallel work.
Exercise 10 connects the solved tree to the mock ROS robot; exercise 11 uses
the TurtleBot world in Gazebo with Nav2. Executable instructor solutions and
verification commands are in [the solutions guide](jupyter_notebooks/exercises/.solutions/README.md).

## Autonomous planning lab

The [Robo-restaurant starter](jupyter_notebooks/robo_restaurant/README.md)
starts with numbered notebooks **12–14 in `exercises/`** and includes a Gazebo
restaurant world and demos on resources, device failures and replanning. The demos run without
ROS. Optional **exercise 15** adds a controlled executor with running, success,
failure and cancellation outcomes. Connecting it to a navigating waiter remains
a later milestone.

## Maintainer image repair

`Dockerfile_patch` adds the NumPy 1.26.4 compatibility repair on the pinned pre-repair
Docker Hub image, reusing its existing base layers. It runs no APT upgrade and
copies no lesson files; Compose supplies lessons from the repository bind mount.
The regular Dockerfiles remain available for a deliberate full rebuild.
The legacy `make release` target performs a full, single-platform build; use the
per-platform repair workflow below to preserve both architectures.

The tested multiarch repair is published as `adohaha/fun_ros:jazzy` and
`adohaha/fun_ros:jazzy-edu-20261009`. The previous image is retained as
`adohaha/fun_ros:jazzy-before-edu-20261009` for rollback. Release digests and
validation results are recorded in [LESSONS_LEARNED.md](LESSONS_LEARNED.md).

Build each supported platform using a native builder or configured ARM emulation:

```bash
docker buildx build --platform linux/amd64 --load -f Dockerfile_patch -t adohaha/fun_ros:jazzy-edu-20261009-amd64 .
docker buildx build --platform linux/arm64 --load -f Dockerfile_patch -t adohaha/fun_ros:jazzy-edu-20261009-arm64 .
```

Before publication, source ROS and run `tools/verify_image_patch.py` as ubuntu in
each candidate. It checks imports, a rendered Matplotlib figure, OpenCV array
processing and the actual rqt Plot plugin. For example:

```bash
docker run --rm --user ubuntu --entrypoint bash \
  -v "$PWD/tools/verify_image_patch.py:/tmp/verify_image_patch.py:ro" \
  adohaha/fun_ros:jazzy-edu-20261009-amd64 \
  -lc 'source /opt/ros/jazzy/setup.bash && python3 /tmp/verify_image_patch.py'
```

Use the arm64 candidate with `--platform linux/arm64` to verify that platform.
Also verify a fresh classroom container's
VNC/Jupyter endpoints and core solved notebooks. Keep both amd64 and arm64 in the
published index, then promote the tested versioned index to `jazzy`.

For students already caching the previous Hub image, only the repair layer is new.
Older cached releases can also need base layers that changed before this repair.
See the student update instructions above for pulling and recreating the container.

---

[Presentation Robot Fun with ROS2 from PyCon PL](https://www.youtube.com/watch?v=K5yGKd7ig7A)
