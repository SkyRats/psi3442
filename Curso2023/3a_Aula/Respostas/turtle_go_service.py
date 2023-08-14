#!/usr/bin/env python

#Import Libraries
import sys
import rospy
from turtlesim.srv import TeleportAbsolute

#A function to execute the service
def turtle_gopose():
    #Start a node in ROS' graph as a client to request a rosservice 
    rospy.init_node('turtle_teleport')
    #Waint until the service be done.
    #Note that the client will be paused here until the service finished.
    rospy.wait_for_service('/turtle1/teleport_absolute')
    try:
        #name /turtle1/teleport_absolute as teleport_turtle      
        teleport_turtle = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        #call teleport_turtle with parameters x=8,y=8,theta=90
        teleport_turtle(8,8,90)
        return 
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    turtle_gopose()
 
