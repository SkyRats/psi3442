#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import SetMode, CommandBool
from sensor_msgs.msg import LaserScan # tipo da mensagem do sensor
from mavros_msgs.msg import State
rospy.init_node("takeoff_land")

# Objetos de comandos e estados
current_pose = PoseStamped()
goal_pose = PoseStamped()
velocity = TwistStamped()
current_state = State()
scan = LaserScan()
rate = rospy.Rate(20)

# Funções de callback
def state_callback(msg):
    global current_state
    current_state = msg

def pose_callback(msg):
    global current_pose
    current_pose = msg

def scan_cb(msg):
    global scan
    scan = msg


# Objetos de Service, Publisher e Subscriber
set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
state_sub = rospy.Subscriber("/mavros/state", State, state_callback)
scan_sub = rospy.Subscriber("/laser/scan", LaserScan, scan_cb)
altura = input("Para qual altura deseja ir: ")

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

print("Takeoff")
goal_pose.pose.position.z = 2
while not rospy.is_shutdown() and abs(goal_pose.pose.position.z - current_pose.pose.position.z) > TOL:
    local_position_pub.publish(goal_pose)
    rate.sleep()

velocity.twist.linear.x = 0
velocity.twist.linear.y = 0
while not rospy.is_shutdown():


    erro = altura - current_pose.pose.position.z
    p = 0.5
    velocity.twist.linear.z = p * erro
    vel_pub.publish(velocity)

    print(scan.ranges[0])
    rate.sleep()