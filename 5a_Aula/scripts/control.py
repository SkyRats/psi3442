#!/usr/bin/env python3

import rospy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import Mavlink
from sensor_msgs.msg import LaserScan
from aula_pkg.msg import control

mavState = State()
goal_pose = PoseStamped()
mavPose = PoseStamped()
laser_data = LaserScan()
detection = control()
speed_cmd = TwistStamped()
speed_cmd.twist.angular.z = 0
speed_cmd.twist.linear.z = 0

detection.cx = 0
detection.width = 0
DIS = 0
ALT = 0.5
TOL = 0.1

def laser_callback(data):
    global laser_data
    laser_data = data


def state_callback(state):
    global mavState
    mavState = state

def local_callback(pose):
    global mavPose
    mavPose.pose.position.x = pose.pose.position.x
    mavPose.pose.position.y = pose.pose.position.y
    mavPose.pose.position.z = pose.pose.position.z

def detection_callback(data): #
    global detection
    detection = data

print("inicializando")

rospy.init_node('control_node', anonymous = True)
rate = rospy.Rate(20)

local_position_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size = 20)

state_sub = rospy.Subscriber("/mavros/state", State, state_callback)
local_atual = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_callback)

arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

laser_sub = rospy.Subscriber("/laser/scan", LaserScan, laser_callback, queue_size=1) #
speed_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10) #

print("preparar takeoff")

#Take off
goal_pose.pose.position.x = 0
goal_pose.pose.position.y = 0
goal_pose.pose.position.z = 0.5

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
for i in range(200):
    local_position_pub.publish(goal_pose)
    rate.sleep()
while not rospy.is_shutdown():

    # Node subscreve no topico conde serao enviadas as distancias da esquerda da imagem até o objeto vermelho(cx) e a largura da imagem(width)
    rospy.Subscriber("control_topic",control,detection_callback, queue_size=10) 
    
    errorx = detection.cx - detection.width/2 # errorx é a distancia do do centro da imagem até o centro do objeto vermelho
    print(errorx)

    # Setar as velocidades angulares(z) em função do errorx  
    speed_cmd.twist.angular.z = -errorx/100 

    # Manter a altitude do drone
    if (mavPose.pose.position.z > ALT+TOL):
        speed_cmd.twist.linear.z -= 0.1
    elif (mavPose.pose.position.z < ALT-TOL):
        speed_cmd.twist.linear.z += 0.1
    else:
        speed_cmd.twist.linear.z = 0

    # Publica a velocidade
    speed_pub.publish(speed_cmd)
    rate.sleep()
    
