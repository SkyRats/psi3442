#!/usr/bin/python3
# -*- coding:utf-8 -*-

# ===========================
# === CONFIGURANDO O NODE ===
# ===========================

import rospy # Biblioteca do ROS para Python

# Classes das mensagens e servicos
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
from mavros_msgs.msg import ExtendedState
from geometry_msgs.msg import TwistStamped

import sys
from select import select
import termios
import tty
import numpy as np
import tf

import subprocess

def getKey():
    timeout = 0.5
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def action():

    key = getKey()

    if key == 'b':
        return "increase_vel"
    elif key == 'v':
        return "decrease_vel"
    elif key == 'u':
        return "forward"
    elif key == 'j':
        return "backward"
    elif key == 'h':
        return "left"
    elif key == 'k':
        return "right"
    elif key == 'a':
        return "rotate_left"
    elif key == 'd':
        return "rotate_right"
    elif key == 'w':
        return "up"
    elif key == 's':
        return "down"
    elif key == 'p':
        return "land"
    elif key == 'c':
        return "centralize baloon"
    else:
        return "hold"
    
def drone_reference(direction_array,yaw):
    theta = yaw 
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                [np.sin(theta), np.cos(theta), 0],
                                [0, 0, 1]])    
    vector = np.matmul(rotation_matrix,direction_array)
    return vector

def centralize_baloon():
    # Run the other script to centralize the baloon
    #subprocess.run(["python3", "baloon_centralize.py"])    
    with open("baloon_centralize.py") as f:
        exec(f.read())
    
# Inicializa o node
rospy.init_node("takeoff_land")

# ================================
# === PUBLISHERS E SUBSCRIBERS ===
# ================================

# Objetos de comandos e estados
current_state = State()
current_pose = PoseStamped()
hold_pose = PoseStamped()
goal_pose = PoseStamped()
extended_state = ExtendedState()
cmd_vel = TwistStamped()

# Frequencia de publicacao do setpoint
rate = rospy.Rate(50)

def multiple_rate_sleep(n):
    for i in range(n):
        rate.sleep()

# Funções de callback
def state_callback(msg):
    global current_state
    current_state = msg

def pose_callback(msg):
    global current_pose
    current_pose = msg

def extended_state_callback(msg):
    global extended_state
    extended_state = msg    

# Objetos de Service, Publisher e Subscriber
arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
state_sub = rospy.Subscriber("/mavros/state", State, state_callback)
pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
extended_state_sub = rospy.Subscriber("/mavros/extended_state", ExtendedState, extended_state_callback)



# =============================
# === PREPARACAO PARA O VOO ===
# =============================

# Espera a conexao ser iniciada
rospy.loginfo("Esperando conexao com FCU")
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
    rospy.loginfo("Alterando para modo Offboard")
    while not rospy.is_shutdown() and current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(1.0)):
        result = set_mode_srv(0, "OFFBOARD")
    rospy.loginfo("Drone em modo Offboard")
else:
    rospy.loginfo("Drone já está em modo Offboard")

# Arma o drone
if (not current_state.armed):
    result = arm(True)
    rospy.loginfo("Armando o drone")
    while not rospy.is_shutdown() and not current_state.armed:
        result = arm(True)
    rospy.loginfo("Drone armado")
else:
    rospy.loginfo("Drone ja armado")



# =============================
# === MOVIMENTACAO DO DRONE ===
# =============================

# Tolerancia de posicionamento
TOL = 0.1

rospy.loginfo("Subindo")
goal_pose.pose.position.z = 5
while not rospy.is_shutdown() and abs(goal_pose.pose.position.z - current_pose.pose.position.z) > TOL:
    local_position_pub.publish(goal_pose)
    rate.sleep()

rospy.loginfo("Esperando")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(5):
    local_position_pub.publish(goal_pose)
    rate.sleep()

rospy.loginfo("Teleop custom mode using Offboard mode activated")
rospy.loginfo("Pressione 'u' para ir para frente")
rospy.loginfo("Pressione 'j' para ir para tras")
rospy.loginfo("Pressione 'h' para ir para esquerda")
rospy.loginfo("Pressione 'k' para ir para direita")
rospy.loginfo("Pressione 'a' para girar para esquerda")
rospy.loginfo("Pressione 'd' para girar para direita")
rospy.loginfo("Pressione 'w' para subir")
rospy.loginfo("Pressione 's' para descer")
rospy.loginfo("Pressione 'p' para pousar")
rospy.loginfo("Pressione 'b' para aumentar a velocidade")
rospy.loginfo("Pressione 'v' para diminuir a velocidade")
rospy.loginfo("Pressione 'c' para centralizar um balao")

land_by_altitude = False
land_keyboard_command_pressed = False

vel = 1.0



while not rospy.is_shutdown() and not land_keyboard_command_pressed and not land_by_altitude:

    hold_pose = current_pose

    yaw = tf.transformations.euler_from_quaternion([current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w])[2]

    if action() ==  "increase_vel":
        if vel + 1.0 < 5.0:
            vel = vel + 1.0
        else:
            vel = 5.0

    
    if action() ==  "decrease_vel":
        if vel - 1.0 > 5.0:
            vel = vel - 1.0
        else:
            vel = 0.0

    if action() == "forward":
        speed_intensity = vel
        direction_array = np.array([1,0,0])
        vel_vector = drone_reference(speed_intensity*direction_array,yaw)
        cmd_vel.twist.linear.x = vel_vector[0]
        cmd_vel.twist.linear.y = vel_vector[1]
        cmd_vel.twist.linear.z = vel_vector[2]
        cmd_vel.twist.angular.z= 0.0
        cmd_vel_pub.publish(cmd_vel)
    elif action() == "backward":
        speed_intensity = vel
        direction_array = np.array([-1,0,0])
        vel_vector = drone_reference(speed_intensity*direction_array,yaw)
        cmd_vel.twist.linear.x = vel_vector[0]
        cmd_vel.twist.linear.y = vel_vector[1]
        cmd_vel.twist.linear.z = vel_vector[2]
        cmd_vel.twist.angular.z= 0.0
        cmd_vel_pub.publish(cmd_vel)
    elif action() == "left":
        speed_intensity = vel
        direction_array = np.array([0,1,0])
        vel_vector = drone_reference(speed_intensity*direction_array,yaw)
        cmd_vel.twist.linear.x = vel_vector[0]
        cmd_vel.twist.linear.y = vel_vector[1]
        cmd_vel.twist.linear.z = vel_vector[2]
        cmd_vel.twist.angular.z= 0.0
        cmd_vel_pub.publish(cmd_vel)
    elif action() == "right":
        speed_intensity = vel
        direction_array = np.array([0,-1,0])
        vel_vector = drone_reference(speed_intensity*direction_array,yaw)
        cmd_vel.twist.linear.x = vel_vector[0]
        cmd_vel.twist.linear.y = vel_vector[1]
        cmd_vel.twist.linear.z = vel_vector[2]
        cmd_vel.twist.angular.z= 0.0
        cmd_vel_pub.publish(cmd_vel)
    elif action() == "rotate_left":  
        cmd_vel.twist.angular.z = 0.5
        cmd_vel.twist.linear.x = 0.0
        cmd_vel.twist.linear.y = 0.0
        cmd_vel.twist.linear.z = 0.0
        cmd_vel_pub.publish(cmd_vel)
    elif action() == "rotate_right":
        cmd_vel.twist.angular.z = -0.5
        cmd_vel.twist.linear.x = 0.0
        cmd_vel.twist.linear.y = 0.0
        cmd_vel.twist.linear.z = 0.0
        cmd_vel_pub.publish(cmd_vel)
    elif action() == "up":
        cmd_vel.twist.linear.z = vel
        cmd_vel.twist.linear.x = 0.0
        cmd_vel.twist.linear.y = 0.0
        cmd_vel.twist.angular.z= 0.0
        cmd_vel_pub.publish(cmd_vel)
    elif action() == "down":
        cmd_vel.twist.linear.z = -vel
        cmd_vel.twist.linear.y = 0.0
        cmd_vel.twist.linear.x = 0.0
        cmd_vel.twist.angular.z= 0.0
        cmd_vel_pub.publish(cmd_vel)
        if(current_pose.pose.position.z < vel):
            land_by_altitude = True
    elif action() ==   "land":
        land_keyboard_command_pressed  = True
    elif action() == "centralize baloon":
        print("Centralizando balao")
        centralize_baloon()
    else: #"hold"
        while action() == "hold" and not rospy.is_shutdown():
            print("Mantendo a posição")
            local_position_pub.publish(hold_pose)
            rate.sleep()
    rate.sleep()


# Coloca no modo Land
if (current_state.mode != "AUTO.LAND"):
    result = set_mode_srv(0, "AUTO.LAND")
    rospy.loginfo("Alterando para modo Land")
    while not rospy.is_shutdown() and current_state.mode != "AUTO.LAND":
        result = set_mode_srv(0, "AUTO.LAND")
    rospy.loginfo("Drone em modo Land")
else:
    rospy.loginfo("Drone ja esta em modo Land")

#Espera Pousar
land_state_on_ground = 1
while not rospy.is_shutdown() and extended_state.landed_state != land_state_on_ground:
    if(extended_state.landed_state == 1):
        rospy.loginfo("Pousado no solo. Altura = " + str(current_pose.pose.position.z) + " m")
    elif(extended_state.landed_state == 2):
        rospy.loginfo("Voando. Altura = " + str(current_pose.pose.position.z) + " m")
    elif(extended_state.landed_state == 3):
        rospy.loginfo("Decolando. Altura = " + str(current_pose.pose.position.z) + " m")
    elif(extended_state.landed_state == 4):
        rospy.loginfo("Pousando. Altura = " + str(current_pose.pose.position.z) + " m") 
    multiple_rate_sleep(10)


#Printa 3 vezes que pousou no solo
i=0
while not rospy.is_shutdown() and i<3:
    if(extended_state.landed_state == 1):
        rospy.loginfo("Pousado no solo. Altura = " + str(current_pose.pose.position.z) + " m")
    elif(extended_state.landed_state == 2):
        rospy.loginfo("Voando. Altura = " + str(current_pose.pose.position.z) + " m")
    elif(extended_state.landed_state == 3):
        rospy.loginfo("Decolando. Altura = " + str(current_pose.pose.position.z) + " m")
    elif(extended_state.landed_state == 4):
        rospy.loginfo("Pousando. Altura = " + str(current_pose.pose.position.z) + " m") 
    i+=1
    multiple_rate_sleep(10)  
       
#Espera o usuario pressionar Ctrl+C
while not rospy.is_shutdown():
    print("Pressione Ctrl+C para encerrar a simulacao")
    multiple_rate_sleep(100)       