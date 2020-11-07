FROM tiryoh/ros2-desktop-vnc:dashing
LABEL maintainer igorzubrycki@gmail.com

ENV USER root
RUN apt-get update
RUN apt-get install -y apt-utils
RUN apt-get upgrade --yes
RUN apt-get install -y python3-pip #
RUN pip3 install notebook
###
ENV USER ubuntu
#from https://automaticaddison.com/how-to-install-and-launch-ros2-using-docker/
RUN mkdir -p ~/turtlebot3_ws/src
WORKDIR /home/ubuntu/turtlebot3_ws
COPY setup.bash /home/ubuntu/setup.bash
RUN ["/bin/bash", "-c", "/home/ubuntu/setup.bash"]
COPY jupyter_notebook_config.py /home/ubuntu/.jupyter/jupyter_notebook_config.py
