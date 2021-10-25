# Aula 6 - Exercício Gazebo e Sensores

Nesta aula, vamos colocar em prática a utilização de sensores na navegação autônoma do drone.

## 1. Lidar

Antes de tudo, precisamos iniciar uma simulação que contenha o LIDAR. Nesse caso, vamos colocar o que queremos inciar em um arquivo launch `iris_lidar_downward.launch`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
    <include file="$(find px4)/launch/mavros_posix_sitl.launch">
        <arg name="vehicle" value="iris"/>
        <arg name="sdf" value="$(find p1_psi3442)/models/iris_lidar_downward/model.sdf"/>
        <arg name="world" value="$(find p1_psi3442)/worlds/mundo.world" />
    </include>
</launch>
```

Podemos perceber que incluímos o nosso drone, um LIDAR e o mundo que vamos rodar a simulação.

Agora, vamos fazer uma aplicação do LIDAR. Nas aulas anteriores você aprendeu a consultar no `rostopic list` em qual tópico estão sendo publicadas as leituras do sensor. Assim, podemos escrever um código que use essas leituras:

```py
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
```
Nesse exemplo, usamos o LIDAR para indicar a distância do drone do chão, ou seja, a sua altura.

> O código acima apresenta apenas a parte de controle do drone, não mostramos os objetos de comando e estado, as funções de callback e os services, publishers e subscribers, pois já foram apresentados exemplos nas aulas anteriores. Caso queira conferir o código completo, olhe o arquivo `lidar.py`.

Com tudo pronto, podemos testar em simulação. Para isso, devemos rodar no terminal:

```sh
cd <caminho_do_seu_catkin_ws>
source src/simulation/scripts/setup.bash

roslaunch <seu_catkin_ws> iris_lidar_downward.launch
```

Isso abrirá a simulação com o seu drone e LDIAR no mundo especificado. Agora, para rodar seu código, basta executar o arquivo `lidar.py`.

*Confira a altura que você gostaria que o drone alcançasse e a leitura retornada pelo LIDAR.*

**Desafio:** Otimizar o controle de altura do drone, alcançando uma precisão maior da altura que deseja alcançar.

## 2. Câmera

Nessa etapa, vamos utilizar a câmera para centralizar o drone a uma plataforma circular laranja.

Antes de tudo, criaremos o nosso launch `iris_fpv_cam.launch` com o drone e uma câmera, além de especificar o mundo que queremos que aconteça a simulação:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
    <include file="$(find px4)/launch/mavros_posix_sitl.launch">
        <arg name="vehicle" value="iris"/>
        <arg name="sdf" value="$(find p1_psi3442)/models/iris_fpv_cam/model.sdf"/>
        <arg name="world" value="$(find p1_psi3442)/worlds/aula.world" />
    </include>
</launch>
```

Agora, para realizar a missão, vamos usar o *OpenCV*, então devemos incluir a biblioteca:

```py
import cv2
from cv_bridge import CvBridge,CvBridgeError
```

> Como no exercício anterior, vamos entrar em detalhe apenas no controle com o sensor. Para conferir o código inteiro, olhe o arquivo `camera.py`.

```py
TOL=0.1
TOL_PIX = 5

print("Takeoff")
goal_pose.pose.position.z = 9
while not rospy.is_shutdown() and abs(goal_pose.pose.position.z - current_pose.pose.position.z) > TOL:
    local_position_pub.publish(goal_pose)
    rate.sleep()

height,width,channels = cv_image.shape
a,b=  height/2, width/2

# Especificações das cores para a criação da máscara
lowerb = np.array([0, 180, 230])
upperb = np.array([20, 255, 255])

while not rospy.is_shutdown():
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lowerb, upperb)
    
    # Mostra a imagem da câmera com a máscara aplicada
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
```

Com o código pronto, vamos iniciar a simulação:

```sh
cd <caminho_do_seu_catkin_ws>
source src/simulation/scripts/setup.bash

roslaunch <seu_catkin_ws> iris_fpv_cam.launch
```

E agora, basta rodar o código de controle `camera.py`.

Observe o drone centralizar a base circular e tente mover essa base novamente para verificar se o drone centraliza novamente.

**Desafio:** Adicione uma base de outra cor e faça o drone centralizar a uma das bases.
