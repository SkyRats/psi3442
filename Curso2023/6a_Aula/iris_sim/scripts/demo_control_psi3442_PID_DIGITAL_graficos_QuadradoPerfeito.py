#!/usr/bin/python3
# -*- coding:utf-8 -*-

#Exemplos PX4 usando mavros
#Exemplo em C++: https://dev.px4.io/v1.11_noredirect/en/ros/mavros_offboard.html
#Exemplo em python: https://docs.px4.io/main/en/ros/mavros_offboard_python.html

#1) Importando bibliotecas Importantes
from json import tool
from re import X
from sqlite3 import Time
from tkinter import E
import time
import rospy #Habilita ROS+pyhton
from geometry_msgs.msg import PoseStamped #Importa biblioteca de menssagens "geometry_msgs" subtipo PoseStamped -> posicao
from mavros_msgs.srv import SetMode, CommandBool #Importa services do node packge mavros
from mavros_msgs.msg import State #Importa biblioteca de mensagens do tipo  mavros_msgs subtipo State
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import Thrust

import numpy as np
import matplotlib.pylab as plt

#Explicacao:
    #SetMode: Service dedicado a setar modo de voo: Manual, Position, Land, Offboard, Onboard
    #CommandBool: Documentacao: http://docs.ros.org/en/hydro/api/mavros/html/srv/CommandBool.html (usaremos para armar o drone)
    #State: Importa biblioteca de mensagens mavros_msgs subtipo State -> Estado do drone: conexão, Armado, Modos de voo

#2) Iniciando esse node python no ROS
rospy.init_node("square_path")
#2) Instanciacao de Objetos de comandos e estados
current_state = State()
current_pose = PoseStamped()
goal_pose = PoseStamped()
vel_pid_cmd = TwistStamped()
attitude_raw = AttitudeTarget()
thrust_cmd = Thrust()

#3) Varivaveis para plotar graficos
coordenada_x = []
coordenada_y = []
coordenada_z = []
tempo = []


#3) Estruturacao python-ROS

    # Funcoes de callback
def state_callback(msg):
    global current_state
    current_state = msg

def pose_callback(msg):
    global current_pose
    current_pose = msg

    # Objetos de Service, Publisher e Subscriber
        #Services da Mavros
arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        #Publishers em topicos da mavros
local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
attitude_cmdvel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped,queue_size=10)
attitude_raw_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget,queue_size=10)
thrust_pub = rospy.Publisher('/mavros/setpoint_attitude/thrust',Thrust, queue_size=10)
        #Subscribers em topicos da mavros
state_sub = rospy.Subscriber("/mavros/state", State, state_callback)
pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)



#4) Frequencia de publicacao do setpoint
rate = rospy.Rate(40)

    #Explicacao: PX4 has a timeout of 500ms between two OFFBOARD commands
    #por isso eh nescessario manter comandos a cada T<timeout. Note que 500ms -> 2Hz e 20Hz eh bem maior (ok!)

################################################################################################################
# 
#  IMPLEMENTACAO DO PID DISCRETO
#
#            e       u
#    r ->( )->[c(z)]->[G(s)]-> y
#      + -^b                |
#         |------------------


#GANHOS QUADRADO 
#Kp_r=5
#Ti_r= 200
#Td_r = 0.001 
#sat_r = 1.5
#Ts = 1/20
#N_r=20

Kp_r = 1.5#2.3
Ti_r= 65
Td_r = 0.5
sat_r = 1.5
Ts = 1/40
N_r=10


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

# ################################################################################################################    

#5) Tempo inicial
t_inicial = float(time.time())

#5) Espera a conexao ser iniciada
print("Esperando conexao com FCU")
while not rospy.is_shutdown() and not current_state.connected:
    rate.sleep()

# Publica algumas mensagens antes de trocar o modo de voo
for i in range(100):
    local_position_pub.publish(goal_pose)
    rate.sleep()

    #Explicacao: A PX4 aceitara comandos recebidos via protocolo MAVLink advindos da MAVROS se houver um fluxo 
    #de dados no sistema. Por isso, mandamos 100 mensagens para criar esse fluxo e acionar o modo Offboard.

# Coloca no modo Offboard
last_request = rospy.Time.now()
if (current_state.mode != "OFFBOARD"):
    result = set_mode_srv(0, "OFFBOARD")
    print("Alterando para modo Offboard")
    while not rospy.is_shutdown() and current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(1.0)):
        result = set_mode_srv(0, "OFFBOARD") #chama o service de set o modo de voo do drone ate que ele seja alterado
    print("Drone em modo Offboard")
    print(result)
else:
    print("Drone ja esta em modo Offboard")

    #Explicacao: Na linha 68 chama-se o service "set_mode_srv". Procurando-se na internet por: "set_mode_service mavros"
    #encontra-se documentacao: http://docs.ros.org/en/noetic/api/mavros_msgs/html/srv/SetMode.html que redireciona para 
    #http://wiki.ros.org/mavros/CustomModes e com um ctrl+f e a palavra "offboard" a forma correta de chamar o servico fica clara.
    #A opcao "OFFBOARD" eh uma custom mode do firmware PX4.


#6) Arma o drone
if (not current_state.armed):
    result = arm(True) 
    print("Armando o drone")
    while not rospy.is_shutdown() and not current_state.armed:
        result = arm(True) #chama o service de armar o drone ate que ele seja armado
    print("Drone armado")
else:
    print("Drone ja armado") 

    #Observacao: Se o drone ja estava armado, isso pode indicar um mal funcionamento ou uma conduta errada por parte do usuario
    #Seria recomendavel a condicao else implicar em arm(False), e reeiniciar o script por questoes de seguranca. 


#7) Comandos de movimentacao
TOL=0.1
    #Toelerancia relativamente baixa mas permite que a trajetoria quadrada seja realizada mais rapidamente. ~ 1 min

    #Subindo
print("Subindo")
goal_pose.pose.position.z = 2
while not rospy.is_shutdown() and abs(goal_pose.pose.position.z - current_pose.pose.position.z) > TOL:
    local_position_pub.publish(goal_pose)
    rate.sleep()    
print("Esperando na coordenada (0,0,2)")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(5):
    local_position_pub.publish(goal_pose)
    #print("realPosition = (%.2f , %.2f , %.2f)" % (goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z ))
    rate.sleep() 

    #Algoritimo simples: Fazer Quadrado com velocidade nula em cada vertice
print("Para frente")
goal_pose.pose.position.x = 5
goal_pose.pose.position.y = 0
goal_pose.pose.position.z = 2
while not rospy.is_shutdown() and abs(goal_pose.pose.position.x - current_pose.pose.position.x) > TOL:
    # vel_pid_cmd.twist.linear.x = control_x.pid(goal_pose.pose.position.x,current_pose.pose.position.x,Kp_s,Ti_s,Td_s,N_s,sat_s,Ts)
    # vel_pid_cmd.twist.linear.y = control_y.pid(goal_pose.pose.position.y,current_pose.pose.position.y,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    # vel_pid_cmd.twist.linear.z = control_z.pid(goal_pose.pose.position.z,current_pose.pose.position.z,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.x = control_x.pid(goal_pose.pose.position.x,current_pose.pose.position.x,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.y = control_y.pid(goal_pose.pose.position.y,current_pose.pose.position.y,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.z = control_z.pid(goal_pose.pose.position.z,current_pose.pose.position.z,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)

    attitude_cmdvel_pub.publish(vel_pid_cmd)
        #Salva p/ plot
    coordenada_x.append(current_pose.pose.position.x)
    coordenada_y.append(current_pose.pose.position.y)
    coordenada_z.append(current_pose.pose.position.z)
    t = float(time.time())
    tempo.append(t - t_inicial)
        #Espera Ts segundos para mandar proximo comando de velocidade  
    rate.sleep()

print("Esperando Esperando na coordenada (5,0,2)")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(5):
    #local_position_pub.publish(current_pose)
    vel_pid_cmd.twist.linear.x = control_x.pid(goal_pose.pose.position.x,current_pose.pose.position.x,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.y = control_y.pid(goal_pose.pose.position.y,current_pose.pose.position.y,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.z = control_z.pid(goal_pose.pose.position.z,current_pose.pose.position.z,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    attitude_cmdvel_pub.publish(vel_pid_cmd)
    #Salva p/ plot
    coordenada_x.append(current_pose.pose.position.x)
    coordenada_y.append(current_pose.pose.position.y)
    coordenada_z.append(current_pose.pose.position.z)
    t = float(time.time())
    tempo.append(t - t_inicial)
        #Espera Ts segundos para mandar proximo comando
    rate.sleep()

print("Para lado y>0")
goal_pose.pose.position.x = 5
goal_pose.pose.position.y = 5
goal_pose.pose.position.z = 2
while not rospy.is_shutdown() and abs(goal_pose.pose.position.y - current_pose.pose.position.y) > TOL:
    # vel_pid_cmd.twist.linear.x = control_x.pid(goal_pose.pose.position.x,current_pose.pose.position.x,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    # vel_pid_cmd.twist.linear.y = control_y.pid(goal_pose.pose.position.y,current_pose.pose.position.y,Kp_s,Ti_s,Td_s,N_s,sat_s,Ts)
    # vel_pid_cmd.twist.linear.z = control_z.pid(goal_pose.pose.position.z,current_pose.pose.position.z,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.x = control_x.pid(goal_pose.pose.position.x,current_pose.pose.position.x,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.y = control_y.pid(goal_pose.pose.position.y,current_pose.pose.position.y,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.z = control_z.pid(goal_pose.pose.position.z,current_pose.pose.position.z,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    attitude_cmdvel_pub.publish(vel_pid_cmd)
        #Salva p/ plot
    coordenada_x.append(current_pose.pose.position.x)
    coordenada_y.append(current_pose.pose.position.y)
    coordenada_z.append(current_pose.pose.position.z)
    t = float(time.time())
    tempo.append(t - t_inicial)
    #print(vel_pid_cmd.twist.linear.x)
        #Espera Ts segundos para mandar proximo comando de velocidade      
    rate.sleep()
print("Esperando Esperando na coordenada (5,5,2)")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(5):
    #local_position_pub.publish(current_pose)
    vel_pid_cmd.twist.linear.x = control_x.pid(goal_pose.pose.position.x,current_pose.pose.position.x,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.y = control_y.pid(goal_pose.pose.position.y,current_pose.pose.position.y,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.z = control_z.pid(goal_pose.pose.position.z,current_pose.pose.position.z,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    attitude_cmdvel_pub.publish(vel_pid_cmd)
    #Salva p/ plot
    coordenada_x.append(current_pose.pose.position.x)
    coordenada_y.append(current_pose.pose.position.y)
    coordenada_z.append(current_pose.pose.position.z)
    t = float(time.time())
    tempo.append(t - t_inicial)
        #Espera Ts segundos para mandar proximo comando
    rate.sleep()    

print("Para tras")
goal_pose.pose.position.x = 0
goal_pose.pose.position.y = 5
goal_pose.pose.position.z = 2
while not rospy.is_shutdown() and abs(goal_pose.pose.position.x - current_pose.pose.position.x) > TOL:
    # vel_pid_cmd.twist.linear.x = control_x.pid(goal_pose.pose.position.x,current_pose.pose.position.x,Kp_s,Ti_s,Td_s,N_s,sat_s,Ts)
    # vel_pid_cmd.twist.linear.y = control_y.pid(goal_pose.pose.position.y,current_pose.pose.position.y,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    # vel_pid_cmd.twist.linear.z = control_z.pid(goal_pose.pose.position.z,current_pose.pose.position.z,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.x = control_x.pid(goal_pose.pose.position.x,current_pose.pose.position.x,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.y = control_y.pid(goal_pose.pose.position.y,current_pose.pose.position.y,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.z = control_z.pid(goal_pose.pose.position.z,current_pose.pose.position.z,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    attitude_cmdvel_pub.publish(vel_pid_cmd)
        #Salva p/ plot
    coordenada_x.append(current_pose.pose.position.x)
    coordenada_y.append(current_pose.pose.position.y)
    coordenada_z.append(current_pose.pose.position.z)
    t = float(time.time())
    tempo.append(t - t_inicial)
        #Espera Ts segundos para mandar proximo comando de velocidade      
    rate.sleep()
print("Esperando Esperando na coordenada (0,5,2)")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(5):
    #local_position_pub.publish(current_pose)
    vel_pid_cmd.twist.linear.x = control_x.pid(goal_pose.pose.position.x,current_pose.pose.position.x,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.y = control_y.pid(goal_pose.pose.position.y,current_pose.pose.position.y,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.z = control_z.pid(goal_pose.pose.position.z,current_pose.pose.position.z,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    attitude_cmdvel_pub.publish(vel_pid_cmd)
    #Salva p/ plot
    coordenada_x.append(current_pose.pose.position.x)
    coordenada_y.append(current_pose.pose.position.y)
    coordenada_z.append(current_pose.pose.position.z)
    t = float(time.time())
    tempo.append(t - t_inicial)
        #Espera Ts segundos para mandar proximo comando
    rate.sleep()    

print("Para lado")
goal_pose.pose.position.x = 0
goal_pose.pose.position.y = 0
goal_pose.pose.position.z = 2
while not rospy.is_shutdown() and abs(goal_pose.pose.position.y - current_pose.pose.position.y) > TOL/10:
    # vel_pid_cmd.twist.linear.x = control_x.pid(goal_pose.pose.position.x,current_pose.pose.position.x,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    # vel_pid_cmd.twist.linear.y = control_y.pid(goal_pose.pose.position.y,current_pose.pose.position.y,Kp_s,Ti_s,Td_s,N_s,sat_s,Ts)
    # vel_pid_cmd.twist.linear.z = control_z.pid(goal_pose.pose.position.z,current_pose.pose.position.z,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.x = control_x.pid(goal_pose.pose.position.x,current_pose.pose.position.x,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.y = control_y.pid(goal_pose.pose.position.y,current_pose.pose.position.y,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.z = control_z.pid(goal_pose.pose.position.z,current_pose.pose.position.z,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    attitude_cmdvel_pub.publish(vel_pid_cmd)
        #Salva p/ plot
    coordenada_x.append(current_pose.pose.position.x)
    coordenada_y.append(current_pose.pose.position.y)
    coordenada_z.append(current_pose.pose.position.z)
    t = float(time.time())
    tempo.append(t - t_inicial)
        #Espera Ts segundos para mandar proximo comando de velocidade  
    rate.sleep()
print("Esperando Esperando na coordenada (0,0,2)")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(5):
    #local_position_pub.publish(current_pose)
    vel_pid_cmd.twist.linear.x = control_x.pid(goal_pose.pose.position.x,current_pose.pose.position.x,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.y = control_y.pid(goal_pose.pose.position.y,current_pose.pose.position.y,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    vel_pid_cmd.twist.linear.z = control_z.pid(goal_pose.pose.position.z,current_pose.pose.position.z,Kp_r,Ti_r,Td_r,N_r,sat_r,Ts)
    attitude_cmdvel_pub.publish(vel_pid_cmd)
    #Salva p/ plot
    coordenada_x.append(current_pose.pose.position.x)
    coordenada_y.append(current_pose.pose.position.y)
    coordenada_z.append(current_pose.pose.position.z)
    t = float(time.time())
    tempo.append(t - t_inicial)
        #Espera Ts segundos para mandar proximo comando
    rate.sleep()       


#8) Coloca no modo Land
if (current_state.mode != "AUTO.LAND"):
    result = set_mode_srv(0, "AUTO.LAND")
    print("Alterando para modo Land")
    while not rospy.is_shutdown() and current_state.mode != "AUTO.LAND":
        result = set_mode_srv(0, "AUTO.LAND")
    print("Drone em modo Land")
else:
    print("Drone ja esta em modo Land")

print("Drone Pousou")    

    #Explicacao: Mandar o drone para a cota z=0 nao funciona pois existem erros de odometria ao longo do percurso e o terreno
    #sobre o qual o drone voa pode ser irregular e conter desniveis. Eh preciso um metodo mais robusto de pouso, utilizamos o
    #metodo AUTO.LAND da PX4: http://wiki.ros.org/mavros/CustomModes


#9 Graficos
#coordenada_x
#coordenada_y
#posicao_xyxy
#tempo
#9.1 Plano XY
plt.figure(1)
plt.plot(coordenada_x,coordenada_y)
plt.plot([0,5,5,0,0],[0,0,5,5,0])
plt.fill_between([0,5],[0+TOL,0+TOL],[0-TOL,0-TOL],facecolor='gray', alpha=0.25)
plt.axvspan(5-TOL,5+TOL, alpha=0.25,ymin=0.07, ymax=0.92, color='gray')
plt.fill_between([5,0],[5+TOL,5+TOL],[5-TOL,5-TOL],facecolor='gray', alpha=0.25)
plt.axvspan(0-TOL,0+TOL, alpha=0.25,ymin=0.07, ymax=0.92, color='gray')
plt.xlabel('x')
plt.ylabel('y')
plt.axis('tight')
plt.show()

#axs.fill_between(t, mu1+sigma1, mu1-sigma1, facecolor='blue', alpha=0.5)

#plt.figure(2)
#plt.plot(tempo,posicao_xyxy)
#plt.xlabel('t [s]')
#plt.ylabel('xyxy [m]')
#plt.axis('tight')
#plt.show()

#9.2 Esforco de Controle posicao_xyxy vs tempo
fig, axs = plt.subplots(3)
fig.suptitle('Coordinates X,Y,Z')
axs[0].plot(tempo, coordenada_x)
axs[0].plot([tempo[0], tempo[len(tempo)-1]],[5, 5])
axs[0].plot([tempo[0], tempo[len(tempo)-1]],[0, 0])
axs[0].set_title('x')
axs[1].plot(tempo, coordenada_y)
axs[1].plot([tempo[0], tempo[len(tempo)-1]],[5, 5])
axs[1].plot([tempo[0], tempo[len(tempo)-1]],[0, 0])
axs[1].set_title('y')
axs[2].plot(tempo, coordenada_z)
axs[2].plot([tempo[0], tempo[len(tempo)-1]],[2, 2])
axs[2].set_title('z')
for ax in axs.flat:
    ax.set(xlabel='time [s]', ylabel='position')
# Hide x labels and tick labels for top plots and y ticks for right plots.
for ax in axs.flat:
    ax.label_outer()
plt.show()