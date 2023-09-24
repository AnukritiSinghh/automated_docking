# Automated Docking
This submission is for final project ENPM673


# To Execute

Dependencies:
- Turtlebot3
- Fudicial_marker

Turtlebot3 installation:
```
cd catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b noetic-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b noetic-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
catkin build
```

installation of arucodetect:
```
sudo apt-get install ros-noetic-aruco-detect
```
installation of fudicial markers(recommended but not required if you have installed arucodetect)
```
cd catkin_ws/src
git clone https://github.com/UbiquityRobotics/fiducials.git
catkin build
```

PACKAGE INSTALLATION INSTRUCTION
download the package in workspace of your choice/src
e.g. download the package in catkin_ws/src
```
cd catkin_ws/src
catkin build autodock_core autodock_examples autodock_sim
source devel/setup.bash
```

PACKAGE RUNNING INSTRUCTION
# case1: To run the basic docking scenario
```
roslaunch autodock_sim tb3_dock_sim.launch
```

in different terminal:
```
 rostopic pub /autodock_action/goal 
autodock_core/AutoDockingActionGoal {} --once
```

# case2: To run the lidar based obstacle avoidance with docking scenario
```
roslaunch autodock_sim tb3_dock_sim.launch
```
in terminal2: 
```
rostopic pub /autodock_action/goal autodock_core/AutoDockingActionGoal {} --once
```
in terminal3: 
```
source ~/catkin_ws/devel/setup.bash 
rosrun autodock_core obstacle_avoidance.py 
```
# case3: To run the perception based obstacle avoidance with docking scenario
```
 roslaunch autodock_sim tb3_dock_sim_cam.launch
 ```
 in terminal2:
 ```
  source ~/catkin_ws/devel/setup.bash 
  rosrun autodock_core action_test.py 
```
If the docking failed, keep running the same action_test.py file again. it will align to seee both markers and dock.

# Link to our Videos:

Result 1: 
The camera is used to make decision to go left or right, when the cam could see all the aruco markers, it will stop turning and wait for command "Ctrl + C" to start. if 
docking failed, run the script again

1) https://drive.google.com/file/d/1gU-ykN2ErM0vu0zqHCw8-xqayuQUUNXK/view?usp=share_link

Result 2: Lidar Detection stop and Docking
2) https://drive.google.com/file/d/1kFU0z21jPpkfyPOoTPXfUItolWa388N1/view?resourcekey

Result 3: Camera Left Turn
3) https://drive.google.com/file/d/1Ol7UcWrtiSVMqgEBOMlOkmFY94aZXwTe/view?resourcekey

Hardware
4) https://drive.google.com/file/d/1zNuYmxqIYEmJy1BLkppNRuWSl--_u3Bd/view?resourcekey

RVIZ Hardware
5) https://drive.google.com/file/d/1rWiEAp69topNlgG3cYIZ0w3uAK5v127W/view?resourcekey

