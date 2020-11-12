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
###
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
