```
git clone <our repo>
mkdir ~/parking_ws/src
cd parking_ws
catkin_make
source devel/setup.bash
```

```
roslaunch aruco pose_estimation.launch 
```

Open new terminal window
```
rosrun aruco pose.py 
```
