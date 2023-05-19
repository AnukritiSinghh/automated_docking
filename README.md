# To Execute

Dependencies:
Turtlebot3
Fudicial_marker

Turtlebot3 installation:

`cd catkin_ws/src`
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b noetic-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b noetic-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
catkin build

installation of arucodetect:
sudo apt-get install ros-noetic-aruco-detect

installation of fudicial markers(recommended but not required if you have installed arucodetect)
cd catkin_ws/src
git clone https://github.com/UbiquityRobotics/fiducials.git
catkin build

PACKAGE INSTALLATION INSTRUCTION
download the package in workspace of your choice/src
e.g. download the package in catkin_ws/src
cd catkin_ws/src
catkin build autodock_core autodock_examples autodock_sim
source devel/setup.bash

PACKAGE RUNNING INSTRUCTION
case1: To run the basic docking scenario
roslaunch autodock_sim tb3_dock_sim.launch
in different terminal: rostopic pub /autodock_action/goal autodock_core/AutoDockingActionGoal {} --once

case2: To run the lidar based obstacle avoidance with docking scenario
roslaunch autodock_sim tb3_dock_sim.launch
in terminal2: rostopic pub /autodock_action/goal autodock_core/AutoDockingActionGoal {} --once
in terminal3: source ~/catkin_ws/devel/setup.bash 
              rosrun autodock_core obstacle_avoidance.py 

# case3: To run the perception based obstacle avoidance with docking scenario
# roslaunch autodock_sim tb3_dock_sim.launch
# in terminal2: rostopic pub /autodock_action/goal autodock_core/AutoDockingActionGoal {} --once
# in terminal3: source ~/catkin_ws/devel/setup.bash 
#               rosrun autodock_core obstacle_avoidance.py 
