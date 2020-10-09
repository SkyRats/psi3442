#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool


rospy.init_node('ComandSquareStateMachine', anonymous = True)
rate = rospy.Rate(20)

ComandPub = rospy.Publisher("MissionFlow", Bool, queue_size=1)

MF = input('enter 1 to end the mission:')
while not rospy.is_shutdown():
    ComandPub.publish(False)
    rate.sleep   