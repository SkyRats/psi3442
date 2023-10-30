#!/usr/bin/python3
# -*- coding:utf-8 -*-

#====================================
#  Libraries
#====================================

import numpy as np
import rospy
import cv2 as cv
from cv_bridge import CvBridge,CvBridgeError

from sensor_msgs.msg import Image

#====================================
# Useful Custom Functions
#====================================
def resized_templates(template,scale_percentages):
    templates = []
    for scale in scale_percentages:
        width = int(template.shape[1] * scale / 100)
        height = int(template.shape[0] * scale / 100)
        dim = (width, height)
            # resize image
        resized = cv.resize(template, dim, interpolation = cv.INTER_AREA)
        templates.append(resized)
    return templates

#====================================
#  ROS Setup
#====================================

rospy.init_node("baloon_detector")

rate = rospy.Rate(60)

# Bridge ros-opencv
bridge_object = CvBridge()

# Post detection image publisher
cv_image = Image()


#-- Get new frame
def camera_callback(message):

    # Bridge de ROS para CV
    cv_image = bridge_object.imgmsg_to_cv2(message, "bgr8")
    #====================================
    #  Computational Vision Algorithm
    #====================================
        #Image from Drone Camera published in ROS topic
    assert cv_image is not None, "file could not be read, check with os.path.exists()"
        #Baloon's template image
    template = cv.imread('template_baloon.png', cv.IMREAD_GRAYSCALE)
    assert template is not None, "file could not be read, check with os.path.exists()"

    cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)

        #Resize template
    scale = 60
    width = int(template.shape[1] * scale / 100)
    height = int(template.shape[0] * scale / 100)
    dim = (width, height)
    template_resized = cv.resize(template, dim, interpolation = cv.INTER_AREA)

    meth = 'cv.TM_CCOEFF_NORMED'
    method = eval(meth)
    # Apply template Matching
    res = cv.matchTemplate(cv_image,template_resized,method)
    min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
    # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
    if method in [cv.TM_SQDIFF, cv.TM_SQDIFF_NORMED]:
        top_left = min_loc
    else:
        top_left = max_loc
    cv.rectangle(cv_image,top_left, (top_left[0] + template_resized.shape[1], top_left[1] + template_resized.shape[0]), 255, 2)
    cv.imshow('Detected',cv_image)
    cv.waitKey(1)

    
        

try:
    print("\nCreating image subscriber...")
    rospy.Subscriber('/iris/usb_cam/image_raw', Image, camera_callback)
    print("Subscriber created!")
except:
    print('Error trying to create subscribers!')

rospy.spin()
