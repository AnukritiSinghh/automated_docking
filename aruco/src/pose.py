#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2.aruco as aruco

K = CameraInfo.K
D = CameraInfo.D
print(K)

def callback(data):
    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Output debugging information to the terminal
    # rospy.loginfo("receiving video frame")
    # Convert ROS Image message to OpenCV image
    
    current_frame = br.imgmsg_to_cv2(data)
    current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    arucoDict = cv2.aruco.Dictionary_get(aruco.DICT_4X4_250)
    arucoParams = cv2.aruco.DetectorParameters_create()
    corners,ids,rejected = cv2.aruco.detectMarkers(current_frame_gray,arucoDict,parameters=arucoParams)
    print("Aruco ID:",ids[0])
    aruco.drawDetectedMarkers(current_frame, corners)
    cv2.imshow("camera", current_frame)

    cv2.waitKey(1)


def listener():
    rospy.init_node("video_sub_py", anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback)

    rospy.spin()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    listener()