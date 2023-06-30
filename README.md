# Fun with ROS 2

Robotics is fun and ROS 2 is a must.

In this workshop, we will play around with ROS 2 and Simulating some robots

You need to have Docker and docker compose installed.
It can be docker engine:
Instructions for Ubuntu are provided [here](https://docs.docker.com/engine/install/ubuntu/)

but for beginners, an easier option might be Docker Desktop: [Windows](https://docs.docker.com/desktop/install/windows-install/) [Linux](https://docs.docker.com/desktop/install/linux-install/), [Mac](https://docs.docker.com/desktop/install/mac-install/)




https://github.com/AdoHaha/ros_fun/assets/2242877/33d32bce-2f59-4905-88df-bea3bd0eb838


 I suggest cloning this repository.


`git clone https://github.com/AdoHaha/ros_fun`

`cd ros_fun`

Use 

`docker compose up ` to run the container.

Also, run 

`docker container exec -it --user ubuntu ros_fun bash -i /home/ubuntu/run_jupyter.sh`

to start jupyter inside the container.

Access the virtual machine screen by navigating to 

[http://localhost:6080](http://localhost:6080)

access the jupyter notebooks by navigating to:

[http://localhost:8888](http://localhost:8888) 

on your **host** machine. 

From there open [*exercises folder*](http://localhost:8888/exercises/1.%20introduction.ipynb) to access introduction

The demo uses ROS Humble

---

<iframe width="560" height="315" src="https://www.youtube.com/embed/K5yGKd7ig7A" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen>Fun with ROS2 presentation from Pycon 2020</iframe>
