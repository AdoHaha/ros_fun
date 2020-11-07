FROM tiryoh/ros2-desktop-vnc:dashing
LABEL maintainer igorzubrycki@gmail.com

ENV USER root
RUN apt-get update
RUN apt-get install -y apt-utils
RUN apt-get upgrade --yes
###
RUN curl -sSL http://get.gazebosim.org | sh # gazebo installation script
RUN apt remove gazebo11 libgazebo11-dev
RUN apt install gazebo9 libgazebo9-dev ros-dashing-gazebo-ros-pkgs
###
RUN apt-get install python3-colcon-common-extensions python3-vcstool
RUN apt-get install ros-dashing-cartographer ros-dashing-cartographer-ros
RUN apt-get install ros-dashing-navigation2 ros-dashing-nav2-bringup
ENV USER ubuntu
#from https://automaticaddison.com/how-to-install-and-launch-ros2-using-docker/
RUN mkdir -p ~/turtlebot3_ws/src
WORKDIR ~/turtlebot3_ws
RUN wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
RUN vcs import ~/turtlebot3_ws/src < turtlebot3.repos

RUN colon build --symlink-install
RUN echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
RUN echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
