#!/usr/bin/env python3
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialArray
# from obs3_neha import vel

global marker_detected
print('Starting')
marker_detected=False 

def callback2(msg):
    for i in range(len(msg.fiducials)):
        print("markers detected")
    # print(msg.fiducials.fiducial_id)
    marker_detected=True
    # move.linear.x = 0.0
    # move.angular.z = 0.0
    # pub.publish(move) 
    # if len(msg.data)>0:
    #     marker_detect=True

def callback(scan_msg):
    # print("inside laser callback",marker_detected)
    if(marker_detected):
        return
    print ('-------------------------------------------')
    print ('Obstacle distance at 0 deg:   {}'.format(scan_msg.ranges[0]))
    print ('Obstacle distance at 15 deg:  {}'.format(scan_msg.ranges[15]))
    print ('Obstacle distance at 345 deg: {}'.format(scan_msg.ranges[345]))
    print ('-------------------------------------------')
    thr1 = 0.8 # Laser scan range threshold
    if(marker_detected):
        return
    if scan_msg.ranges[0]<=thr1 or scan_msg.ranges[15]<=thr1 or scan_msg.ranges[-15]<=thr1:
        move.linear.x = 0.0 
        pub.publish(move) 


move = Twist() 
rospy.init_node('obstacle_avoidance_node') # Initializes a node
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
sub = rospy.Subscriber("/scan", LaserScan, callback)
sub2 = rospy.Subscriber("/fiducial_vertices", FiducialArray, callback2)
rospy.spin() 