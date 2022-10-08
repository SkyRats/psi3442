#!/usr/bin/python3
# -*- coding:utf-8 -*-

#Exemplos PX4 usando mavros
#Exemplo em C++: https://dev.px4.io/v1.11_noredirect/en/ros/mavros_offboard.html
#Exemplo em python: https://docs.px4.io/main/en/ros/mavros_offboard_python.html

#1) Importando bibliotecas Importantes
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
rate = rospy.Rate(20)

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

# Global Variables 
e = 0
e_i = 0
un_1 = 0
uIn_1 = 0
uDn_1 = 0
yn_1 = 0

#GANHOS QUADRADO PERFEITO
#Kp=0.5
#Ti= 200
#Td = 0.001 #ganho Td >= 0.01 nao funciona e o drone para em (1.4,0,2) quando tenta ir pra frente
#sat = 4
#Ts = 1/20
#N=10

#GANHOS QUADRADO SUBAMORTECIDO
Kp=1.1
Ti= 10
Td = 0.001 #ganho Td >= 0.01 nao funciona e o drone para em (1.4,0,2) quando tenta ir pra frente
sat = 4
Ts = 1/20
N=10

def pid(r,yn,Kp,Ti,Td,sat):
    global e
    global e_i
    global un_1 
    global uIn_1 
    global uDn_1 
    global yn_1 
    e = r - yn
    #anti-windup
    if (un_1 > sat or un_1 < - sat):
        e_i = 0
    else:
        e_i = e
    #pid
    uP = Kp*e
    uIn = uIn_1 + (Kp*Ts/Ti)*e_i #backward
    uDn = Td/(Td + N*Ts)*uDn_1 - (Kp*N*Td)/(Td+N*Ts)*(yn-yn_1)
    un = uP + uIn + uDn
    #update past values variables
    un_1 = un
    uIn_1 = uIn
    uDn_1 = uDn
    yn_1 = yn
    #return
    return un

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
    vel_pid_cmd.twist.linear.x = pid(goal_pose.pose.position.x,current_pose.pose.position.x,Kp,Ti,Td,sat)
    vel_pid_cmd.twist.linear.y = pid(goal_pose.pose.position.y,current_pose.pose.position.y,Kp,Ti,Td,sat)
    vel_pid_cmd.twist.linear.z = pid(goal_pose.pose.position.z,current_pose.pose.position.z,Kp,Ti,Td,sat)
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
    #local_position_pub.publish(goal_pose)
    local_position_pub.publish(current_pose)
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
    #local_position_pub.publish(goal_pose)
    vel_pid_cmd.twist.linear.x = pid(goal_pose.pose.position.x,current_pose.pose.position.x,Kp,Ti,Td,sat)
    vel_pid_cmd.twist.linear.y = pid(goal_pose.pose.position.y,current_pose.pose.position.y,Kp,Ti,Td,sat)
    vel_pid_cmd.twist.linear.z = pid(goal_pose.pose.position.z,current_pose.pose.position.z,Kp,Ti,Td,sat)
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
    #local_position_pub.publish(goal_pose)
    local_position_pub.publish(current_pose)
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
    vel_pid_cmd.twist.linear.x = pid(goal_pose.pose.position.x,current_pose.pose.position.x,Kp,Ti,Td,sat)
    vel_pid_cmd.twist.linear.y = pid(goal_pose.pose.position.y,current_pose.pose.position.y,Kp,Ti,Td,sat)
    vel_pid_cmd.twist.linear.z = pid(goal_pose.pose.position.z,current_pose.pose.position.z,Kp,Ti,Td,sat)
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
    #local_position_pub.publish(goal_pose)
    local_position_pub.publish(current_pose)
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
while not rospy.is_shutdown() and abs(goal_pose.pose.position.y - current_pose.pose.position.y) > TOL:
    vel_pid_cmd.twist.linear.x = pid(goal_pose.pose.position.x,current_pose.pose.position.x,Kp,Ti,Td,sat)
    vel_pid_cmd.twist.linear.y = pid(goal_pose.pose.position.y,current_pose.pose.position.y,Kp,Ti,Td,sat)
    vel_pid_cmd.twist.linear.z = pid(goal_pose.pose.position.z,current_pose.pose.position.z,Kp,Ti,Td,sat)
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
    #local_position_pub.publish(goal_pose)
    local_position_pub.publish(current_pose)
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
plt.xlabel('x')
plt.ylabel('y')
plt.axis('tight')
plt.show()

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