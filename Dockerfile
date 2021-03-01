FROM tiryoh/ros-desktop-vnc:noetic
LABEL maintainer igorzubrycki@gmail.com

ENV USER root
RUN apt-get update
RUN apt-get install -y apt-utils
RUN apt-get upgrade --yes
RUN apt-get install -y python3-pip #
RUN pip3 install --upgrade pip
RUN pip3 install notebook
RUN pip3 install RISE
RUN pip3 install opencv-contrib-python ipywidgets
RUN apt-get --yes install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt-image-view \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers  \
  ros-noetic-teb-local-planner
#ADD ./jupyter_notebooks/rviz_nav.rviz /opt/ros/noetic/share/nav2_bringup/rviz/nav2_default_view.rviz
ENV USER ubuntu
#from https://automaticaddison.com/how-to-install-and-launch-ros2-using-docker/
RUN mkdir -p /home/ubuntu/catkin_ws/src
WORKDIR /home/ubuntu/catkin_ws
RUN git clone https://github.com/AWegierska/multirobot_nav.git /home/ubuntu/catkin_ws/src/multirobot_nav
RUN git clone https://github.com/AWegierska/pkg_tsr.git /home/ubuntu/catkin_ws/src/pkg_tsr
ADD ./jupyter_notebooks /home/ubuntu/catkin_ws/src/jupyter_notebooks
ADD ./run_jupyter.sh /home/ubuntu/run_jupyter.sh
COPY setup.bash /home/ubuntu/setup.bash
RUN ["/bin/bash", "-c", "/home/ubuntu/setup.bash"]
COPY jupyter_notebook_config.py /home/ubuntu/.jupyter/jupyter_notebook_config.py
RUN jupyter-nbextension install rise --py --sys-prefix
RUN apt-get --yes install gedit
RUN pip3 install widgetsnbextension
RUN jupyter nbextension install --user --py widgetsnbextension
RUN jupyter nbextension enable widgetsnbextension --user --py
ADD ./jupyter_notebooks/Jupyter.desktop /home/ubuntu/Desktop/Jupyter.desktop
