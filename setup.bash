source /opt/ros/noetic/setup.bash
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh

chmod 755 ./install_ros_noetic.sh 
bash ./install_ros_noetic.sh
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
sudo apt install ros-noetic-dynamixel-sdk --yes
sudo apt install ros-noetic-turtlebot3-msgs --yes
sudo apt install ros-noetic-turtlebot3 --yes
sudo apt install ros-noetic-turtlebot3-gazebo --yes
#echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
jupyter notebook --generate-config
cd /home/ubuntu/catkin_ws && catkin_make
