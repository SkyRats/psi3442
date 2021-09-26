#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
    
rospy.init_node("takeoff_land")

# Objetos de comandos e estados
current_state = State()
current_pose = PoseStamped()
goal_pose = PoseStamped()

# Funções de callback
def state_callback(msg):
    global current_state
    current_state = msg

def pose_callback(msg):
    global current_pose
    current_pose = msg

# Objetos de Service, Publisher e Subscriber
arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
state_sub = rospy.Subscriber("/mavros/state", State, state_callback)
pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)

# Frequência de publicação do setpoint
rate = rospy.Rate(20)

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

# Comandos de movimentação
TOL=0.1

print("Subindo")
goal_pose.pose.position.z = 2
while not rospy.is_shutdown() and abs(goal_pose.pose.position.z - current_pose.pose.position.z) > TOL:
    local_position_pub.publish(goal_pose)
    rate.sleep()

print("Esperando")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(5):
    local_position_pub.publish(goal_pose)
    rate.sleep()

print("Para frente")
goal_pose.pose.position.x = 5
while not rospy.is_shutdown() and abs(goal_pose.pose.position.x - current_pose.pose.position.x) > TOL:
    local_position_pub.publish(goal_pose)
    rate.sleep()

print("Esperando")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(5):
    local_position_pub.publish(goal_pose)
    rate.sleep()

# Coloca no modo Land
if (current_state.mode != "AUTO.LAND"):
    result = set_mode_srv(0, "AUTO.LAND")
    print("Alterando para modo Land")
    while not rospy.is_shutdown() and current_state.mode != "AUTO.LAND":
        result = set_mode_srv(0, "AUTO.LAND")
    print("Drone em modo Land")
else:
    print("Drone já está em modo Land")