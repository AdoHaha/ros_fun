FROM tiryoh/ros2-desktop-vnc:foxy
LABEL maintainer igorzubrycki@gmail.com

ENV USER root
RUN apt-get update
RUN apt-get install -y apt-utils
RUN apt-get upgrade --yes
RUN apt-get install -y python3-pip #
RUN pip3 install --upgrade pip
RUN pip3 install notebook
RUN pip3 install RISE
RUN pip3 install opencv-contrib-python
RUN apt-get install ros-foxy-cartographer --yes
RUN apt-get install ros-foxy-cartographer-ros --yes
RUN apt-get install ros-foxy-navigation2 --yes
RUN apt-get install ros-foxy-nav2-bringup --yes
RUN apt-get install ros-foxy-gazebo-* --yes
RUN apt-get install ros-foxy-dynamixel-sdk ros-foxy-turtlebot3-msgs ros-foxy-turtlebot3 ros-foxy-turtlebot3-simulations --yes
ADD ./jupyter_notebooks/rviz_nav.rviz /opt/ros/foxy/share/nav2_bringup/rviz/nav2_default_view.rviz
ENV USER ubuntu
#from https://ubuntu.com/blog/simulate-the-turtlebot3
RUN mkdir -p ~/turtlebot3_ws/src
WORKDIR /home/ubuntu/turtlebot3_ws
ADD ./jupyter_notebooks /home/ubuntu/turtlebot3_ws/src/jupyter_notebooks
ADD ./run_jupyter.sh /home/ubuntu/run_jupyter.sh
COPY setup.bash /home/ubuntu/setup.bash
RUN ["/bin/bash", "-c", "/home/ubuntu/setup.bash"] #
COPY jupyter_notebook_config.py /home/ubuntu/.jupyter/jupyter_notebook_config.py
RUN jupyter-nbextension install rise --py --sys-prefix
#ADD ./turtlebot3_lds_2d.lua /home/ubuntu/turtlebot3_ws/src/turtlebot3/turtlebot3_cartographer/turtlebot3_lds_2d.lua
RUN pip3 install --upgrade flask
