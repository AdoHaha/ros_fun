FROM tiryoh/ros2-desktop-vnc:jazzy
LABEL maintainer="igorzubrycki@gmail.com"

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV TURTLEBOT3_MODEL=waffle_pi
ENV DISPLAY=:1.0

USER root

RUN apt-get update -q && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
      apt-utils \
      jupyter-notebook \
      python3-pip \
      python3-ipywidgets \
      python3-opencv \
      python3-bqplot \
      python3-flask \
      python3-natsort \
      ros-jazzy-cartographer \
      ros-jazzy-cartographer-ros \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-ros-gz \
      ros-jazzy-dynamixel-sdk \
      ros-jazzy-turtlebot3-msgs \
      ros-jazzy-turtlebot3 \
      ros-jazzy-turtlebot3-simulations \
      ros-jazzy-turtlebot3-gazebo && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# RISE is not packaged in Ubuntu Noble. Keep Notebook from apt so ROS Python
# packages remain on the system interpreter, and install only the extension.
RUN python3 -m pip install --break-system-packages --no-cache-dir RISE

RUN mkdir -p /home/ubuntu/turtlebot3_ws/src /home/ubuntu/.jupyter && \
    chown -R ubuntu:ubuntu /home/ubuntu/turtlebot3_ws /home/ubuntu/.jupyter

WORKDIR /home/ubuntu/turtlebot3_ws

COPY --chown=ubuntu:ubuntu ./jupyter_notebooks /home/ubuntu/turtlebot3_ws/src/jupyter_notebooks
COPY --chown=ubuntu:ubuntu ./run_jupyter.sh /home/ubuntu/run_jupyter.sh
COPY --chown=ubuntu:ubuntu ./setup.bash /home/ubuntu/setup.bash
COPY --chown=ubuntu:ubuntu ./jupyter_notebook_config.py /home/ubuntu/.jupyter/jupyter_notebook_config.py
COPY ./jupyter.supervisor.conf /etc/supervisor/conf.d/jupyter.conf

USER ubuntu

RUN /home/ubuntu/setup.bash

USER root

RUN jupyter-nbextension install rise --py --sys-prefix && \
    jupyter nbextension enable rise --py --sys-prefix && \
    jupyter nbextension enable --py widgetsnbextension

USER root
