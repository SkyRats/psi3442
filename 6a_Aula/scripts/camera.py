#!/usr/bin/python
# -*- coding:utf-8 -*-

import numpy as np
import rospy
import cv2

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
rospy.init_node("takeoff_land")

# Objetos de comandos e estados
current_pose = PoseStamped()
goal_pose = PoseStamped()
velocity = TwistStamped()
current_state = State()
rate = rospy.Rate(60)

# Funções de callback
def state_callback(msg):
    global current_state
    current_state = msg

def pose_callback(msg):
    global current_pose
    current_pose = msg


def camera_callback(data):
    bridge_object = CvBridge()
    global cv_image
    cv_image = bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")


# Objetos de Service, Publisher e Subscriber
set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
state_sub = rospy.Subscriber("/mavros/state", State, state_callback)
image_sub = rospy.Subscriber("/iris/usb_cam/image_raw", Image, camera_callback)

# Espera a conexão ser iniciada
print("Esperando conexão com FCU")
while not rospy.is_shutdown() and not current_state.connected:
    rate.sleep()

# Publica algumas mensagens antes de trocar o modo de voo
for i in range(100):
    local_position_pub.publish(goal_pose)
    rate.sleep()

# Coloca no modo Offboard
last_request = rospy.Time.now()
if (current_state.mode != "OFFBOARD"):
    result = set_mode_srv(0, "OFFBOARD")
    print("Alterando para modo Offboard")
    while not rospy.is_shutdown() and current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(1.0)):
        result = set_mode_srv(0, "OFFBOARD")
    print("Drone em modo Offboard")
else:
    print("Drone já está em modo Offboard")

# Arma o drone
if (not current_state.armed):
    result = arm(True)
    print("Armando o drone")
    while not rospy.is_shutdown() and not current_state.armed:
        result = arm(True)
    print("Drone armado")
else:
    print("Drone já armado")


TOL=0.1
TOL_PIX = 5

print("Takeoff")
goal_pose.pose.position.z = 9
while not rospy.is_shutdown() and abs(goal_pose.pose.position.z - current_pose.pose.position.z) > TOL:
    local_position_pub.publish(goal_pose)
    rate.sleep()


height,width,channels = cv_image.shape
a,b=  height/2, width/2

lowerb = np.array([0, 180, 230])
upperb = np.array([20, 255, 255])


while not rospy.is_shutdown():
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lowerb, upperb)
    cv2.imshow("Mask",mask)
    cv2.waitKey(3)

    M = cv2.moments(mask)
    if (M['m00'] != 0 ):
        a = int(M['m10']/M['m00'])
        b = int(M['m01']/M['m00'])
        erro_a = (width/2) -  a 
        erro_b = (height/2) - b  

        p = 0.005

        if abs(erro_a) > TOL_PIX:
            velocity.twist.linear.y = erro_a * p
        else:
            velocity.twist.linear.y = 0

        if abs(erro_b) > TOL_PIX:
            velocity.twist.linear.x = erro_b * p
        else:
            velocity.twist.linear.x = 0

    else:
        print("Estou perdido")
        velocity.twist.linear.y = 0
        velocity.twist.linear.x = 0

    if velocity.twist.linear.x > 1:
       velocity.twist.linear.x =1
    if velocity.twist.linear.x < -1:
        velocity.twist.linear.x=-1
    if velocity.twist.linear.y > 1:
        velocity.twist.linear.y =1
    if velocity.twist.linear.y < -1:
        velocity.twist.linear.y =-1

    
    vel_pub.publish(velocity)

    if abs(erro_a) < TOL_PIX and abs(erro_b) < TOL_PIX:
        print("Estou centralizado")
    else:
        print("Vel x: " + str(velocity.twist.linear.x)) 
        print("Vel y: " + str(velocity.twist.linear.y))

    rate.sleep()