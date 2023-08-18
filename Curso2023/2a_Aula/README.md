2 aula

## Install ROS Debian 10 in Labrador - CITI USP

```
roscore
python -V
python3 -V
lsb_release -a
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop
sudo apt update
sudo apt install ros-noetic-desktop
source /opt/ros/noetic/setup.bash
roscore
nano ~./bashrc
nano ~/.bashrc
source /opt/ros/noetic/setup.bash
rosrun turtlesim turtlesim_node 
source /opt/ros/noetic/setup.bash
rosrun turtlesim turtle_teleop_key 
roscore
rosrun turtlesim turtlesim node
rosrun turtlesim turtlesim_node 
mkdir -p ros_workspaces/turtle_ws/src
cd ros_workspaces/turtle_ws/
catkin build - j2
ls
cd src
ls
catkin_init_workspace
cd ..
catkin build - j2
catkin build -j2
cd
sudo apt-get install ros-noetic-catkin
cd ros_workspaces/turtle_ws/
catkin build -j2
cd
pip3 install -U catkin_tools
pip3 -v
sudo apt update
sudo apt install python3-pip
wich pip3
pip3 install -U catkin_tools
cd ros_workspaces/
ls
cd turtle_ws/
ls
catkin build -j2
catkin build - j2
ls
cd
pip3 --version
sudo apt install python3-catkin-tools python3-osrf-pycommon
sudo apt-get update && sudo apt-get install build-essential
cd ros_workspaces/turtle_ws/
ls
catkin build -j2
ls
cd
nano ~/.bashrc
cd ~/ros_workspaces/turtle_ws/src
catkin_create_pkg turtle_control rospy roscpp
cd
cd ~/ros_workspaces/turtle_ws/src/turtle_control
mkdir scripts
touch turtle_go_topic.py
touch turtle_go_service.py
ls
cd scripts/
touch turtle_go_topic.py
touch turtle_go_service.py
ls
cd
cd ~/ros_workspaces/turtle_ws/src/turtle_control/scripts
chmod +x turtle_go_topic.py
chmod +x turtle_go_service.py
ls
cd
cd ~/ros_workspaces/turtle_ws/
source devel/setup.bash
ls
cd
roscd turtle_control
cd ros_workspaces/turtle_ws/
catkin build -j2
source devel/setup.bash
cd
roscd turtle_control
rosrun turtle_control turtle_go_topic.py 
cd
rosrun turtle_control turtle_go_topic.py 
cd
roscore
rosrun turtle_control turtle_go_topic.py 
cd ros_workspaces/turtle_ws/src/turtle_control/
ls
cd scripts/
python3 turtle_go_topic.py 
cd
rosrun turtle_control turtle_go_topic.py 
history
```

