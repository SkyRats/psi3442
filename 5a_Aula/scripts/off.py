#!/usr/bin/env python3

import rospy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import Mavlink

mavState = State()
goal_pose = PoseStamped()
mavPose = PoseStamped()



def state_callback(state):
    global mavState
    mavState = state

def local_callback(pose):
    global mavPose
    mavPose.pose.position.x = pose.pose.position.x
    mavPose.pose.position.y = pose.pose.position.y
    mavPose.pose.position.z = pose.pose.position.z


print("inicializando")

rospy.init_node('FirstTry', anonymous = True)
rate = rospy.Rate(20)

local_position_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size = 20)

state_sub = rospy.Subscriber("/mavros/state", State, state_callback)
local_atual = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_callback)

arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

print("preparar takeoff")

#Take off
goal_pose.pose.position.x = 0
goal_pose.pose.position.y = 0
goal_pose.pose.position.z = 2

for i in range(100):
    local_position_pub.publish(goal_pose)
    rate.sleep()
print("posicao enviada")

while mavState.mode != "OFFBOARD":
    result = set_mode_srv(0, "OFFBOARD")
    rate.sleep()

print("offboard enable")

while not (mavState.armed):
    arm(True)
    rate.sleep()


print("mav armado")
print("iniciando missao")

while not rospy.is_shutdown():
    local_position_pub.publish(goal_pose)
    print("we are: {x: " + str(mavPose.pose.position.x) + ", y: " + str(mavPose.pose.position.y) + ", z: " + str(mavPose.pose.position.z))
    print("we are: {x: " + str(goal_pose.pose.position.x) + ", y: " + str(goal_pose.pose.position.y) + ", z: " + str(goal_pose.pose.position.z))
    rate.sleep()
    