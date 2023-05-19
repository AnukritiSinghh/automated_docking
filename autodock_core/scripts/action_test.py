#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialArray,FiducialTransformArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess


class ObstacleAvoidance:
    def __init__(self):
        self.marker_detected = False
        self.vel = 0.0
        self.start_laser = True
        self.start_image = False
        self.move = Twist()

        rospy.init_node('obstacle_avoidance_node')  # Initializes a node

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        # self.sub_aruco = rospy.Subscriber("/fiducial_vertices", FiducialArray, self.aruco_callback)
        self.sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback, queue_size=1)

    # def calculate_distance(self,area):
    #     reference_area = 100.0  # Known area of the fiducial at a reference distance
    #     reference_distance = 1.0  # Reference distance at which the fiducial is observed with the reference area
    #     estimated_distance = reference_distance * (reference_area / area) ** 0.5
    #     return estimated_distance

    def aruco_callback(self, msg):
        # for i in range(len(msg.fiducials)):
        #     print(msg.fiducials[i].fiducial_id)
        if len(msg.fiducials) == 3 and msg.fiducials[1].size == 0.6:
            self.marker_detected = True
            self.move.linear.x = 0.0
            self.move.angular.z = 0.0
            self.pub.publish(self.move)

    def laser_callback(self, scan_msg):
        if not self.start_laser:
            return

        print('-------------------------------------------')
        print('Obstacle distance at 0 deg:   {}'.format(scan_msg.ranges[0]))
        print('Obstacle distance at 15 deg:  {}'.format(scan_msg.ranges[15]))
        print('Obstacle distance at 345 deg: {}'.format(scan_msg.ranges[345]))
        print('-------------------------------------------')

        thr1 = 0.6  # Laser scan range threshold

        if self.marker_detected:
            return

        if (scan_msg.ranges[0] > thr1 and scan_msg.ranges[15] > thr1 and scan_msg.ranges[-15] > thr1):
            self.move.linear.x = 0.08
            self.move.angular.z = 0.0
        else:
            self.move.linear.x = 0.0
            # self.start_image = True
            # if not self.start_image:
            #     self.start_image = True
            #     return
            # if (scan_msg.ranges[0] > thr1 and scan_msg.ranges[15] > thr1 and scan_msg.ranges[-15] > thr1):
            #     self.move.linear.x = 0.2
            #     self.move.angular.z = 0.0
        self.pub.publish(self.move)

    def image_callback(self, img_data):
        if not self.start_image:
            return
        if self.marker_detected:
            return
        rospy.loginfo("Received a new image!")

        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img_data, "passthrough")
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 59, 0])
        upper_red = np.array([179, 255, 255])
        imagemask = cv2.inRange(img, lower_red, upper_red)
        result = cv2.resize(imagemask, (640, 480))
        cv_image = cv2.resize(cv_image, (640, 480))
        result = cv2.line(result, (320, 0), (320, 480), (0, 255, 0), 2)
        # cv2.imshow("mask", result)
        contours, _ = cv2.findContours(result, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print("Number of contours found =", len(contours))
        for c in contours: cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)
        # cv2.imshow("Image window", cv_image)
        if len(contours) >= 2:
            area1 = cv2.contourArea(contours[0])
            area2 = cv2.contourArea(contours[1])
            area = area1 + area2
            print("Contour 1 area:", area1)
            print("Contour 2 area:", area2)

            if area1 > 0 or area2 > 0:
                if area1 > area / 2:
                    print("Right")
                    self.move.angular.z = -1.2
                else:
                    print("Left")
                    self.move.angular.z = 1.2

                self.start_laser = True
                self.start_image = False
            else:
                print("No valid contours, stopping the robot")
                self.start_laser = True
                self.start_image = False
                self.move.angular.z = 0.0

        else:
            print("Contours not found, stopping the robot")
            if(self.marker_detected == False):
                self.start_laser = True
                self.start_image = False
                self.move.angular.z = -0.1

        self.move.linear.x = 0.0
        self.pub.publish(self.move)


    def run(self):
        rospy.spin()

    def publish_to_ros(self,topic,message_type,message):
        command = f"rostopic pub {topic} {message_type} {message} --once"
        subprocess.call(command, shell=True)



if __name__ == '__main__':
    obstacle_avoidance = ObstacleAvoidance()
    obstacle_avoidance.run()
    topic = "/autodock_action/goal"
    message_type = "autodock_core/AutoDockingActionGoal"
    message = {}
    obstacle_avoidance.publish_to_ros(topic, message_type, message)

