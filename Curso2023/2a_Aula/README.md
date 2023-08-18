2 aula

## Install ROS Debian 10 in Labrador - CITI USP

    1  roscore
    2  python -V
    3  python3 -V
    4  lsb_release -a
    5  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    6  sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    7  sudo apt update
    8  sudo apt install ros-noetic-desktop
    9  sudo apt update
   10  sudo apt install ros-noetic-desktop
   11  source /opt/ros/noetic/setup.bash
   12  roscore
   13  nano ~./bashrc
   14  nano ~/.bashrc
   15  source /opt/ros/noetic/setup.bash
   16  rosrun turtlesim turtlesim_node 
   17  source /opt/ros/noetic/setup.bash
   18  rosrun turtlesim turtle_teleop_key 
   19  roscore
   20  rosrun turtlesim turtlesim node
   21  rosrun turtlesim turtlesim_node 
   22  mkdir -p ros_workspaces/turtle_ws/src
   23  cd ros_workspaces/turtle_ws/
   24  catkin build - j2
   25  ls
   26  cd src
   27  ls
   28  catkin_init_workspace
   29  cd ..
   30  catkin build - j2
   31  catkin build -j2
   32  cd
   33  sudo apt-get install ros-noetic-catkin
   34  cd ros_workspaces/turtle_ws/
   35  catkin build -j2
   36  cd
   37  pip3 install -U catkin_tools
   38  pip3 -v
   39  sudo apt update
   40  sudo apt install python3-pip
   41  wich pip3
   42  pip3 install -U catkin_tools
   43  cd ros_workspaces/
   44  ls
   45  cd turtle_ws/
   46  ls
   47  catkin build -j2
   48  catkin build - j2
   49  ls
   50  cd
   51  pip3 --version
   52  sudo apt install python3-catkin-tools python3-osrf-pycommon
   53  sudo apt-get update && sudo apt-get install build-essential
   54  cd ros_workspaces/turtle_ws/
   55  ls
   56  catkin build -j2
   57  ls
   58  cd
   59  nano ~/.bashrc
   60  cd ~/ros_workspaces/turtle_ws/src
   61  catkin_create_pkg turtle_control rospy roscpp
   62  cd
   63  cd ~/ros_workspaces/turtle_ws/src/turtle_control
   64  mkdir scripts
   65  touch turtle_go_topic.py
   66  touch turtle_go_service.py
   67  ls
   68  cd scripts/
   69  touch turtle_go_topic.py
   70  touch turtle_go_service.py
   71  ls
   72  cd
   73  cd ~/ros_workspaces/turtle_ws/src/turtle_control/scripts
   74  chmod +x turtle_go_topic.py
   75  chmod +x turtle_go_service.py
   76  ls
   77  cd
   78  cd ~/ros_workspaces/turtle_ws/
   79  source devel/setup.bash
   80  ls
   81  cd
   82  roscd turtle_control
   83  cd ros_workspaces/turtle_ws/
   84  catkin build -j2
   85  source devel/setup.bash
   86  cd
   87  roscd turtle_control
   88  rosrun turtle_control turtle_go_topic.py 
   89  cd
   90  rosrun turtle_control turtle_go_topic.py 
   91  cd
   92  roscore
   93  rosrun turtle_control turtle_go_topic.py 
   94  cd ros_workspaces/turtle_ws/src/turtle_control/
   95  ls
   96  cd scripts/
   97  python3 turtle_go_topic.py 
   98  cd
   99  rosrun turtle_control turtle_go_topic.py 
  100  history


