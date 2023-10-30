#!/usr/bin/python3
# -*- coding:utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# ROS node
rospy.init_node('simple_image_show_node', anonymous=False)

# Bridge ros-opencv
bridge_object = CvBridge()

# Post detection image publisher
cam = Image()


#-- Get new frame
def camera_callback(message):

    # Bridge de ROS para CV
    cam = bridge_object.imgmsg_to_cv2(message, "bgr8")
    # Show image
    cv2.imshow("ImageFromCV", cam)
    cv2.waitKey(1)

try:
    print("\nCreating image subscriber...")
    rospy.Subscriber('/iris/usb_cam/image_raw', Image, camera_callback)
    print("Subscriber created!")
except:
    print('Error trying to create subscribers!')

frame = None

rospy.spin()

