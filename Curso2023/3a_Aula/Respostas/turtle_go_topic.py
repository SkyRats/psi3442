#!/usr/bin/env python

#Libraries
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class Turtle:
    def __init__(self):
        self.pose = Pose()
        self.vel = Twist()
        self.goal_pose = Pose()

        rospy.init_node("gotogoal")
        self.rate = rospy.Rate(10) # 10Hz
        self.velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 10)
        self.pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)    

    def pose_callback(self,data):
        self.pose.x = data.x
        self.pose.y = data.y
        self.pose.theta = data.theta

    def distance(self, pose, goal_pose):
        return math.sqrt((goal_pose.x - pose.x) * (goal_pose.x - pose.x) + (goal_pose.y - pose.y) * (goal_pose.y - pose.y)) 

    def angle(self, pose, goal_pose):
        return math.atan2(goal_pose.y - pose.y, goal_pose.x - pose.x)

    def moveToGoal(self):
        self.goal_pose.x = float(input("Set your x goal: "))
        self.goal_pose.y = float(input("Set your y goal: "))

        while abs(self.angle(self.pose, self.goal_pose) - self.pose.theta) >= 0.05 and not rospy.is_shutdown():
            self.vel.angular.z = 1
            self.velocity_publisher.publish(self.vel)

        self.vel.angular.z = 0
        while self.distance(self.pose, self.goal_pose) >= 0.3 and not rospy.is_shutdown():

            # Set velocity
            self.vel.linear.x = 1

            self.velocity_publisher.publish(self.vel)
            self.rate.sleep()

        # When get out, is near the goal pose
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.velocity_publisher.publish(self.vel)
        #print(vel.angular.z)

    #=== end class ====

if __name__ == '__main__':

    turtle = Turtle()
    turtle.moveToGoal()
