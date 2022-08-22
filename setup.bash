source /opt/ros/foxy/setup.bash
#wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
#vcs import /home/ubuntu/turtlebot3_ws/src < turtlebot3.repos
#cd /home/ubuntu/turtlebot3_ws/src/ && git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd /home/ubuntu/turtlebot3_ws  && colcon build --symlink-install
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
#echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
jupyter notebook --generate-config
