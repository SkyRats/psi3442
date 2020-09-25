# Aula 05

### Objetivo

Implementar sensores no drone dentro do ambiente de simulação Gazebo.

### Atividade 0 - Clonar o repositório com os packages necessários

Primeiramente abra o seu Terminal e vá até o diretório **src** do seu **Workspace**

Se você seguiu os passos de instalação que se encontram na documentação oficial da MAVROS, provavelmente o seu workspace se chama "catkin_ws". Se esse for o seu caso, rode o seguinte comando no Terminal:

```bash
cd catkin_ws/src
```

clone o reposótório **Simulation** que se encontra no *GitHub* da Skyrats.

```bash
git clone https://github.com/SkyRats/simulation
```

Agora volte na raiz do workspace e compile-o com `catkin_build`:

```bash
cd  ..
```

e depois 

```bash
catkin build
```

## Atividade 1 - Iniciar Simulação

Para iniciar a simulação teríamos que rodar alguns comando para setar varáveis e endereços, e rodar comandos dos ros. Porém nós já criamos um script que automatiza isso tudo e nos facilita a focar no objetivo principal de hoje.

Para rodar o script de simulação, vá até o diretório de *simulation* e depis entre em *scripts*

```bash
cd ~/catkin_ws/src/simulation/scripts
```

E antes de conseguirmos rodar o script, temos que deixá-lo executável:

```bash
chmod +x simulate.sh
```

Agora rode o script em bash

```bash
rosrun simulate simulate.sh
```

Nesta aula nós não iremos utilizar todos os arquivos .launch disponíveis nesse package, portanto siga os seguintes passos:

- Digite 1 para escolher a opção do script onde ele te pergunta qual launch file você deseja rodar
- Digite 7 para escolher o arquivo "irirs_fpv_cam.launch"

O seu terminal deve estar asism o o da imagem abaixo:

![Terminal simulate.sh](images/terminal_1.jpeg)

## Atividade 2 - Criar e executar um node ROS

Para criar um package ROS, e neste caso, o package que iremos utilizar nesta aula, rode os seguintes comandos no seu Terminal:

```bash
cd ~/catkin_ws/src
```

e depois

```bash
catkin_create_pkg aula_pkg mavros_msgs rospy roscpp geometry_msgs sensor_msgs geographic_msgs cv_bridge message_generation
```

Agora, se vc der `ls` no seu Terminal vc deve observar que um package com nome "aula_pkg" foi criado. E se vc entrar no repositório e abrir o *CMakelists.txt*, você deve observar que os parâmetro que utilizamos no comando acima foram adicinados as dependências que serão necessárias no nosso pacote (rospy, roscpp, geometry_msgs, sensor_msgs, geographic_msgs, cv_bridge, message_generation)

Agora, vamos criar um repositório para colcoarmos todos os nossos scripts, para isso rode no seu Terminal:

```bash
cd ~/catkin_ws/src
```

e depois

```bash
mkdir scripts
```

Agora vamos adicionar um script nesse repositório, para isso abra um editor de texto de sua preferência (os comandos abaixo serão com o Gedit):

```bash
gedit avoid.py
```

Agora cole o seguinte código no arquivo vazio que abriu:

```python
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

mavState = State()
goal_pose = PoseStamped()
mavPose = PoseStamped()
laser_data = LaserScan()
speed_cmd = TwistStamped()

DIS = 1
TOL = 0.2
ALT = 0.5

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


print("inicializando")

rospy.init_node('control_node', anonymous = True)
rate = rospy.Rate(20)

local_position_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size = 20)

state_sub = rospy.Subscriber("/mavros/state", State, state_callback)
local_atual = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_callback)

arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

laser_sub = rospy.Subscriber("/laser/scan", LaserScan, laser_callback, queue_size=1)
speed_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

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

    # Mantem a altura
    if (mavPose.pose.position.z > ALT+TOL):
        speed_cmd.twist.linear.z -= 0.1
    elif (mavPose.pose.position.z < ALT-TOL):
        speed_cmd.twist.linear.z += 0.1
    else:
        speed_cmd.twist.linear.z = 0 

    # Detecta proximidade de objetos em 4 direçoes
    if (laser_data.ranges[0] < DIS):
        #Atras
        speed_cmd.twist.linear.x += 0.1
    if (laser_data.ranges[89] < DIS):
        #Esquerda
        speed_cmd.twist.linear.y += 0.1
    if (laser_data.ranges[179] < DIS):
        #Frente
        speed_cmd.twist.linear.x -= 0.1
    if (laser_data.ranges[269] < DIS):
        #Direita
        speed_cmd.twist.linear.y -= 0.1
    if (laser_data.ranges[0] > DIS and laser_data.ranges[89] > DIS and laser_data.ranges[179] > DIS and laser_data.ranges[269] > DIS):
        speed_cmd.twist.linear.x = 0
        speed_cmd.twist.linear.y = 0
    
    # Publica os valores de velocidade
    speed_pub.publish(speed_cmd)
    rate.sleep()
```

Pronto, depois de salvar o arquivo você poderá notar que temos o script `avoid.py`. Esse script faz com que o drone se afaste de objetos que forem detecados pelo Lidar 360.

Com a simulação aberta, vamos agora executar o script.

Primeiro temos que deixar o nosso cript executável:

```bash
chmod +x avoid.py
```

e depois vamos executá-lo com:

```bash
rosrun aula_pkg avoid.py
```

## Atividade 3 - Red Object Detection

O objetivo deste node é utilizar-se do Open CV para fazer com o que o drone detecte um objeto vermelho e realize o movimento de *YAW* para centralizar o mesmo objeto na imagem captada pela sua câmera.

Antes de tudo é necessário certificar que o OpenCV está corretamente instalado na sua máquina.

Para instalar o OpenCV rode o seguinte comando:

```bash
pip install opencv-python
```

Agora vamos precisar fazer o mesmo procedimento que fizemos acima, de criar scripts e deixá-los executáveis, e fazer isso para os segunites scripts:

- control.py
- detection.py

O código do control.py é este:

```python
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
```

O código do detection.py é este:

```python
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from aula_pkg.msg import control
import cv2
import numpy as np
class Follower(object):
    def __init__(self):
        # Node subscreve no topico em que sao recebidas as imagens fa camera frontal do drone
        self.image_sub = rospy.Subscriber("/iris/usb_cam/image_raw", Image, self.camera_callback, queue_size=10)
        
        
        self.bridge_object = CvBridge()

        # Node passa a publicar no topico "control_topic" com o tipo de mensagem control
        self.pub = rospy.Publisher('control_topic', control, queue_size=10)
    def camera_callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        print("Imagem Carregada")
        height,width,channels = cv_image.shape
        descentre = 160
        rows_to_watch = 60
        #crop_image = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        lowerb = np.array([0, 70, 50])
        upperb = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lowerb, upperb)
        #cv2.imshow('mask',mask)
        #cv2.waitKey(0)
        m = cv2.moments(mask, False)
        try:
            cx,cy = m["m10"]/m["m00"],m["m01"]/m["m00"]
        except ZeroDivisionError:
            cy,cx = height/2,width/2

        print(cx)
        
        # Cria a mensagem e seta os valores
        public = control()
        public.cx = cx
        public.width = width

        # Publica a mensagem
        self.pub.publish(public)

def main():
    # Inicia no node
    rospy.init_node('detection_node', anonymous = True)
    follower_object = Follower()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")



if __name__=="__main__":
    main()
```

Depois de criar os dois arquivos dentro de `simulation/scripts` deixe-os executáveis com:

```bash
chmod +x control.py
```

e 

```bash
chmod +x detection.py
```