#!/usr/bin/python3
# -*- coding:utf-8 -*-

#====================================
#  Libraries
#====================================

import rospy
from std_msgs.msg import Int64
import time
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
import numpy as np

#rospy.init_node("baloon_centralizer")

rate = rospy.Rate(60)

#Instanciacao de Objetos de comandos e estados
current_pose = PoseStamped()
vel_pid_cmd = TwistStamped()
horizontal_error  = -8
vertical_error = -8

def timeout(t0,int_timeout,timeout_msg):
    if(rospy.Time.now() - t0 > rospy.Duration(int_timeout)):
        print("timeout achived in " + str(timeout_msg))
        return True
    else:
        return False

def drone_reference(direction_array,yaw):
    theta = yaw 
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                [np.sin(theta), np.cos(theta), 0],
                                [0, 0, 1]])    
    vector = np.matmul(rotation_matrix,direction_array)
    return vector     

    #Subscribers

def pose_callback(msg):
    global current_pose
    current_pose = msg    

#Subscribe  in rostopic 'baloon_tracking/horizontal_error'
def callback_horizontal_error(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global horizontal_error
    horizontal_error = data.data
    #-- Do something with horizontal_error

#Subscribe  in rostopic 'baloon_tracking/vertical_error'
def callback_vertical_error(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global vertical_error
    vertical_error = data.data
    #-- Do something with vertical_error

#Subscribe to rostopic 'baloon_tracking/horizontal_error'
rospy.Subscriber("baloon_tracking/horizontal_error", Int64, callback_horizontal_error)

#Subscribe to rostopic 'baloon_tracking/vertical_error'
rospy.Subscriber("baloon_tracking/vertical_error", Int64, callback_vertical_error)

#Subscribe to rostopic 'mavros/local_position/pose'
pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)

    #Publishers
attitude_cmdvel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped,queue_size=10)

Kp_h = 0.1/2
Ti_h= 100000000
Td_h = 0.2
sat_h = 0.5
Ts = 1/60
N_h=10

Kp_v = 0.02
Ti_v= 150
Td_v = 0.2
sat_v = 1.5
Ts = 1/60
N_v=10

class Control:
    def __init__(self):    
        self.e = 0
        self.e_i = 0
        self.un_1 = 0
        self.uIn_1 = 0
        self.uDn_1 = 0
        self.yn_1 = 0
        self.uP = 0
        self.uIn = 0
        self.uDn = 0
        self.un = 0

    def pid(self,r,yn,Kp,Ti,Td,N,sat,Ts):

        self.e = r - yn
        #anti-windup
        if (self.un_1 > sat or self.un_1 < - sat):
            self.e_i = 0
        else:
            self.e_i = self.e
        #pid
        self.uP = Kp*self.e
        self.uIn = self.uIn_1 + (Kp*Ts/Ti)*self.e_i #backward
        self.uDn = Td/(Td + N*Ts)*self.uDn_1 - (Kp*N*Td)/(Td+N*Ts)*(yn-self.yn_1)
        self.un = self.uP + self.uIn + self.uDn
        #update past values variables
        self.un_1 = self.un
        self.uIn_1 = self.uIn
        self.uDn_1 = self.uDn
        self.yn_1 = yn
        #return
        return self.un

    def pd(self,r,yn,Kp,Td,N,Ts):

        self.e = r - yn
        #pd
        self.uP = Kp*self.e
        self.uDn = Td/(Td + N*Ts)*self.uDn_1 - (Kp*N*Td)/(Td+N*Ts)*(yn-self.yn_1)
        self.un = self.uP + self.uDn
        #update past values variables
        self.un_1 = self.un
        self.uDn_1 = self.uDn
        self.yn_1 = yn
        #return
        return self.un

#Instanciando controles
control_x = Control()
control_y = Control()
control_z = Control()

#Tempo inicial
t_inicial = float(time.time())


TOLhorizontal_error = 1 #%Tolerancia de erro horizontal
TOLvertical_error = 1 #%Tolerancia de erro vertical 

    #Centralize baloon
print("Centralizing Baloon")
#horizontal_error = 0
t0 = rospy.Time.now()
while not rospy.is_shutdown() and abs(horizontal_error) > TOLhorizontal_error and not timeout(t0,5,"horizontal adjustment"):
    yaw = tf.transformations.euler_from_quaternion([current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w])[2]
    
    speed_intensity = control_x.pid(0,horizontal_error,Kp_h,Ti_h,Td_h,N_h,sat_h,Ts)
    direction_array = np.array([0,speed_intensity,0])

    vel_vector = drone_reference(direction_array,yaw)
    
    vel_pid_cmd.twist.linear.x = vel_vector[0]
    vel_pid_cmd.twist.linear.y = vel_vector[1]
    vel_pid_cmd.twist.angular.z= 0.0
    attitude_cmdvel_pub.publish(vel_pid_cmd)
    rate.sleep()

vel_pid_cmd.twist.linear.x=0
vel_pid_cmd.twist.linear.y=0
baloon_lost = False
vertical_error_1 = vertical_error

#vertical_error = 0
t0 = rospy.Time.now()
while not rospy.is_shutdown() and abs(vertical_error) > TOLvertical_error and not baloon_lost and not timeout(t0,8,"vertical adjustment"):
    vel_pid_cmd.twist.linear.z = control_z.pid(0,vertical_error,Kp_v,Ti_v,Td_v,N_v,sat_v,Ts)
    if abs(vertical_error - vertical_error_1) > 3:
        baloon_lost = True
        vel_pid_cmd.twist.linear.z=0
        attitude_cmdvel_pub.publish(vel_pid_cmd)
    else:    
        vertical_error_1 = vertical_error
        attitude_cmdvel_pub.publish(vel_pid_cmd) 
    rate.sleep() 


print("Baloon Centralized")
print("Final Horizontal error in %: ", horizontal_error)
print("Final Vertical error in %: ", vertical_error)
