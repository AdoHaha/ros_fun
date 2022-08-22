# Fun with ROS 2

Robotics are fun and ROS 2 is a must.

In this presentation I present a way to start with ROS 2 using Python: 100 % Python and Jupyter based.

You need to have Docker and docker-compose installed. I suggest cloning this repository.


`git clone https://github.com/AdoHaha/ros_fun`
`cd ros_fun`

Use 

`docker-compose up ` to run the container.

Also, run 

`docker container exec -it --user ubuntu ros_fun bash -i /home/ubuntu/run_jupyter.sh`

to start jupyter inside the container.

Access the virtual machine screen by navigating to 

[http://localhost:6080](http://localhost:6080)

access the jupyter notebooks by navigating to:

[http://localhost:8888](http://localhost:8888) 

on your **host** machine. 

From there open [*fun_with_ROS_2.ipynb*](http://localhost:8888/fun_with_ROS_2.ipynb) to access the presentation and demos.

The demo uses ROS Foxy
