FROM tiryoh/ros2-desktop-vnc:humble
LABEL maintainer igorzubrycki@gmail.com

ENV USER root
RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get upgrade -y && \
    apt-get autoclean && \
    apt-get autoremove
RUN apt-get install -y apt-utils
RUN apt-get install -y --reinstall python3-pip
RUN pip3 install --upgrade pip
RUN pip3 install -U notebook
RUN pip3 install -U RISE ipywidgets
RUN pip3 install opencv-contrib-python
RUN apt-get install ros-humble-cartographer --yes
RUN apt-get install ros-humble-cartographer-ros --yes
RUN apt-get install ros-humble-navigation2 --yes
RUN apt-get install ros-humble-nav2-bringup --yes
RUN apt-get install ros-humble-gazebo-* --yes
RUN apt-get install ros-humble-dynamixel-sdk ros-humble-turtlebot3-msgs ros-humble-turtlebot3 ros-humble-turtlebot3-simulations ros-humble-turtlebot3-gazebo --yes
ADD ./jupyter_notebooks/rviz_nav.rviz /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
RUN apt install chromium-browser -y

ENV USER ubuntu

#from https://ubuntu.com/blog/simulate-the-turtlebot3
RUN mkdir -p ~/turtlebot3_ws/src
WORKDIR /home/ubuntu/turtlebot3_ws
ADD ./jupyter_notebooks /home/ubuntu/turtlebot3_ws/src/jupyter_notebooks
ADD ./run_jupyter.sh /home/ubuntu/run_jupyter.sh
COPY setup.bash /home/ubuntu/setup.bash
RUN ["/bin/bash", "-c", "/home/ubuntu/setup.bash"]
COPY jupyter_notebook_config.py /home/ubuntu/.jupyter/jupyter_notebook_config.py
RUN jupyter-nbextension install rise --py --sys-prefix
RUN jupyter nbextension enable rise --py --sys-prefix
RUN jupyter nbextension enable --py widgetsnbextension
RUN pip3 install -U --ignore-installed flask
#ADD ./turtlebot3_lds_2d.lua /home/
#RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> /home/ubuntu/.bashrc
#RUN  echo "export TURTLEBOT3_MODEL=waffle_pi" >> /home/ubuntu/.bashrc
#RUN echo "source /opt/ros/humble/setup.bash" >> /home/ubuntu/.bashrc
#RUN echo "GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> /home/ubuntu/.bashrc
#ADD ./turtlebot3_lds_2d.lua /home/
ADD ./bashrc /home/ubuntu/.bashrc

RUN cd /home/ubuntu/turtlebot3_ws/src && git clone https://github.com/ros-planning/navigation2.git --branch humble
RUN . /opt/ros/humble/setup.sh && rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble

RUN cd src && git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install
