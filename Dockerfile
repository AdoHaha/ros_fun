FROM tiryoh/ros2-desktop-vnc:dashing
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
RUN apt-get install ros-dashing-cartographer --yes
RUN apt-get install ros-dashing-cartographer-ros --yes
RUN apt-get install ros-dashing-navigation2 --yes
RUN apt-get install ros-dashing-nav2-bringup --yes
ADD ./rviz_nav.rviz /opt/ros/dashing/share/nav2_bringup/rviz/nav2_default_view.rviz
ENV USER ubuntu
#from https://automaticaddison.com/how-to-install-and-launch-ros2-using-docker/
RUN mkdir -p ~/turtlebot3_ws/src
WORKDIR /home/ubuntu/turtlebot3_ws
ADD ./jupyter_notebooks /home/ubuntu/turtlebot3_ws/src/jupyter_notebooks
ADD ./run_jupyter.sh /home/ubuntu/run_jupyter.sh
COPY setup.bash /home/ubuntu/setup.bash
RUN ["/bin/bash", "-c", "/home/ubuntu/setup.bash"]
COPY jupyter_notebook_config.py /home/ubuntu/.jupyter/jupyter_notebook_config.py
RUN jupyter-nbextension install rise --py --sys-prefix
ADD ./turtlebot3_lds_2d.lua /home/ubuntu/turtlebot3_ws/src/turtlebot3/turtlebot3_cartographer/turtlebot3_lds_2d.lua
