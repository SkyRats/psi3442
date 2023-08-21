# Simulação da PX4 e MAVROS

Nesta atividade, iremos explorar a relação entre os softwares executados no sistema embarcado de um drone através de um estudo em simulação.

> *Observação*: O diretório `mav_control` funciona como um package ROS, isto é, se colocado dentro da pasta `src` de um catkin workspace, seus programas poderão ser acessados por comandos ROS como `rosrun`

## 1. PX4

A PX4 é um Firmware de piloto automático open-source amplamente utilizado para drones. Do site do PX4:

> "PX4 is an open source flight control software for drones and other unmanned vehicles."

O PX4 fornece controle de voo e integração para diversos tipos de veículos, como drones, aviões, VTOLs, rovers, barcos, etc. Para isso, ele usa uma base comum de código que é igual para qualquer tipo de "airframe".

O PX4 é organizado para ser um sistema reativo (veja o [Reactive Manifesto](https://www.reactivemanifesto.org/)), ou seja, ele é organizado em várias partes separadas e relativamente independentes (módulos), que se comunicam entre si através de um sistema assíncrono de mensagens ([uORB](https://docs.px4.io/main/en/middleware/uorb.html)), sendo capaz de se adaptar a cargas de trabalho variáveis. Dessa forma, os vários módulos podem ser desenvolvidos e trocados independentemente, reduzindo o risco de danificar outros componentes do sistema e tornando o sistema mais robusto a falhas.

![PX4_Architecture](imgs/PX4_Architecture.fa89af6b.svg)

Ele é executado dentro de uma controladora de voo, como a Pixhawk, em um sistema operacional de tempo real (RTOS) como o NuttX. Assim, ele controla diretamente os motores e recebe informações de sensores como IMU e GPS, assim como comandos diretos de radiocontrole. Porém, ele também é capaz de trocar informações e receber comandos de fontes externas (como um computador no solo ou embarcado no próprio drone), através do protocolo MAVLink.

### 4.1 Arquitetura de Controle PX4
* [Attitude Control](https://en.wikipedia.org/wiki/Attitude_control) se refere tipicamente a orientação de uma aeronave em relação a um determinado eixo de coordenadas utilizada como referência para seu movimento. 

* Altitude Control refere-se simplismente ao controle de altitude de uma aeronave.

* Rate controller: é o controlador de nível mais baixo desenvolvido para o sistema quadrotor. Este controlador recebe as taxas angulares desejadas do controlador de atitude. Em seguida, calcula o erro entre as taxas desejadas e as taxas medidas pelo giroscópio. Este erro é então usado para calcular as três entradas de controle de momento. Estes são combinados com a entrada de controle de empuxo calculada pelo controlador de altitude que são então convertidas para as velocidades desejadas do motor e enviadas aos motores através dos ESCs.

## 2. MAVLink e MAVROS

MAVLink é um protocolo de comunicação leve usado para a comunicação entre a PX4 e outras aplicações como o QGroundControl e o ROS, enviando um conjunto de mensagens e microsserviços para transporte de dados e comandos.

![px4_arch_fc_companion](imgs/px4_arch_fc_companion.c430665d.svg)

Na imagem acima, vemos um esquema de comunicação padrão para drones. A controladora de voo (como a Pixhawk) executa o PX4 se comunica diretamente os motores e sensores. Para aplicações de mais alto nível, frequentemente temos um computador auxiliar para realizar tarefas mais complexas. Esse computador pode ser tanto embarcado, como uma Raspberry Pi ou uma Odroid conectado a uma porta serial da controladora, quanto um notebook ou desktop conectado por telemetria. Em ambos os casos, a comunicação é feita através do protocolo MAVLink. 

Dentro do computador auxiliar, utilizamos o ROS para realizar as tarefas. Assim, é necessário o uso do pacote MAVROS, o qual cria a ponte entre o ROS e a PX4 ao criar uma interface no ROS (através de tópicos e serviços) para a passagem de mensagens MAVLink para a controladora de voo. Com ele, podemos controlar o drone através de nodes do ROS rodando nesse computador auxiliar.

## 3. Simulação no Gazebo

É possível simular o funcionamento dos drones na PX4 através do programa de simulação Gazebo. 

![px4_sitl_overview](imgs/px4_sitl_overview.d5d197f2.svg)

A imagem acima ilustra a estrutura de comunicação da simulação *Software-in-the-loop* (em oposição ao *Hardware-in-the-loop*), na qual todos os componentes são simulados em software. Assim, o Firmware da PX4 roda diretamente em um computador Linux, simulando o processamento da controladora de voo. A interação dessa com o mundo externo através dos motores e sensores é feita pelo simulador Gazebo. Já a comunicação com programas externos (os únicos que, de fato, seriam executados no computador em um sistema real) continua sendo feita através de MAVLink.

Para abrir uma simulação no Gazebo, é necessário clonar o repositório do [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) na pasta `~/src` e executar o seguinte comando do repositório:

```bash
make px4_sitl gazebo
```

Após compilado o Firmware, o Gazebo abre com um drone. Nesse ponto, estamos simulando o drone físico com uma controladora de voo rodando PX4.

Para se comunicar com o ROS, é necessário usar o MAVROS, com o comando

```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

> Se fosse um drone real, trocariamos o parâmetro `fcu_url`, por exemplo, pela porta USB na qual está conectada a telemetria.

## 4. Exemplo: decolagem e pouso

O programa `takeoff_land.py`, localizado na pasta `scripts`, realiza a atividade de decolar um drone, movimentá-lo 5 metros para frente, e pousá-lo, para demonstrar o funcionamento da camada de comunicação e controle Offboard. Esse programa, escrito em Python, funciona como um node no ROS, se comunicando com o drone através da MAVROS. Analisamos em seguida o funcionamento do código.

### 4.1. Configurando o node

```python
#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
```

A biblioteca `rospy` é responsável pela comunicação do programa com o ROS, enquanto que as demais correspondem a classes de mensagens e serviços que serão utilizados. Vale notar que a linha `#!/usr/bin/python` é necessária para que o ROS saiba como executar esse programa.

```python
rospy.init_node("takeoff_land")
```

Declaramos para o ROS que esse programa é um node, com o nome `takeoff_land`. 

```python
# Objetos de comandos e estados
current_state = State()
current_pose = PoseStamped()
goal_pose = PoseStamped()
```

Criamos objetos para cada mensagem que iremos enviar ou receber através dos tópicos do ROS.

> **Revisão**: cada mensagem é definida como uma estrutura de dados contendo alguns campos pré-determinados. Elas são usadas para transmitir informaões entre nodes através de tópicos. Abaixo, temos um exemplo do conteúdo da mensagem `PoseStamped`.
```bash
$ rosmsg show geometry_msgs/PoseStamped
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
```

```python
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
```

Em sequência, criamos os objetos diretamente responsáveis pela comunicação com o ROS: os Publishers, Subscribers e Services. 

> **Revisão**: Os Publishers enviam mensagens de um dado tipo em um determinado tópico. Essas mensagens são recebidas pelos Subscribers inscritos nesse tópico, caracterizando uma comunicação assíncrona. Os Services são canais de comunicação síncronos, feitos diretamente entre nodes sem o intermédio de um tópico.

Nesse caso, precisamos de Services para armar o drone e para alterar seu modo de voo. Temos também um Publisher, utilizado para fornecer ao drone uma posição de destino. Finalmente, nos inscrevemos em dois tópicos: um para receber uma realimentação do estado do drone (armado, modo de voo) e outro para saber sua posição atual. Lembre que Subscribers sempre precisam de uma função de callback para atualizar o estado das variáveis.

### 4.2. Modificando o estado do drone

```python
# Frequência de publicação do setpoint
rate = rospy.Rate(20)
```

É necessário manter um fluxo constante de comandos para o drone. Para isso, utilizamos um objeto `Rate` a uma frequência de 20 Hz.

```python
# Espera a conexão ser iniciada
print("Esperando conexão com FCU")
while not rospy.is_shutdown() and not current_state.connected:
    rate.sleep()
```

Uma vez executando o código, é necessário esperar que o drone esteja de fato conectado para começar a mandar instruções. Fazemos isso através da variável `current_state`, atualizada em um dos Subscribers definidos anteriormente.

```python
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
```

Em seguida, é necessário mudar o modo de voo do drone para Offboard. Este é o único modo de voo no qual o drone executa comandos recebidos através da comunicação MAVLink. Por motivos de segurança, a PX4 só permite que o sistema entre em modo Offboard se houver um fluxo de comandos, logo mandamos 100 mensagens previamente.

> *Observação*: Um modo de voo define como o drone se comporta e como ele reage a estimulos externos. Alguns modos comuns são Manual (controle totalmente manual por radiocontrole), Position (controle por radiocontrole com estabilização de posição), Land (pouso automático) e Offboard (controle via MAVLink).

Para alterar o modo, utilizamos o Service definido no objeto `set_mode_srv`, mas só seguimos quando tivermos certeza de que o drone estiver no modo desejado.


```python
# Arma o drone
if (not current_state.armed):
    result = arm(True)
    print("Armando o drone")
    while not rospy.is_shutdown() and not current_state.armed:
        result = arm(True)
    print("Drone armado")
else:
    print("Drone já armado")
```

Para armar o drone, fazemos um processo parecido com a mudança de modo de voo: chamar um serviço até que o estado desejado seja atingido. 

> **Revisão**: Dizemos que um drone está armado quando os motores estão energizados e podem receber comandos de movimentação. Não se deve se aproximar de um drone armado, mesmo pousado, pois suas hélices podem começar a se movimentar.

### 4.3. Movimentando o drone

```python
# Comandos de movimentação
TOL=0.1

print("Subindo")
goal_pose.pose.position.z = 2
while not rospy.is_shutdown() and abs(goal_pose.pose.position.z - current_pose.pose.position.z) > TOL:
    local_position_pub.publish(goal_pose)
    rate.sleep()
```

Com o drone no modo Offboard e armado, ele finalmente responde a comandos de posição. Assim, publicamos continuamente mensagens no tópico correspondente até que ele atinja a posição desejada (ou suficientemente próximo dela).

```python
print("Esperando")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(5):
    local_position_pub.publish(goal_pose)
    rate.sleep()
```

Para manter o drone parado, é importante manter o fluxo de mensagens. Caso contrário, a PX4 interpreta que a comunicação foi interrompida e reverte para o modo de voo anterior ao Offboard, e perdemos o controle autônomo do drone. 

```python
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
```

Finalmente, realizamos mais comandos de movimentação para que o drone realize a atividade desejada. Note que para pousá-lo, utilizamos o modo AUTO.LAND (pouso automático). Fazemos isso pois nem sempre o chão está em $z=0$, devido a irregularidades no relevo e erros na medição de altitude. Esse modo lida com esses problemas, descendo o drone até que ele atinja o chão.

### 4.4. Criando o ambiente de desenvolvimento

Crie um [workspace](https://github.com/SkyRats/psi3442/tree/master/Curso2023/3a_Aula#1-criando-ros-workspace) chamado drone_ws.

Em seguida crie, dentro do workspace drone_ws, um [ros packge](https://github.com/SkyRats/psi3442/tree/master/Curso2023/3a_Aula#2-criando-ros-package) chamado takeoffandlanding_demo.

Dentro do packge crie um script python chamado ```takeoff_land.py```
que cotenha as instruções para armar, decolar e pousar um drone do gazebo provido pela simulação software in the loop do firmware PX4 com o gazebo e a MAVROS.

Dica: Use os links fornecidos nessa seção para ajudar a criar o ambiente de programação compatível com o ROS.


### 4.5. Executando o código

Terminal 1
```bash
cd src/Firmware/
make px4_sitl gazebo
```
Terminal 2
```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```
Terminal 3
Para executar o código, utilizamos o comando
```bash
rosrun mav_control takeoff_land.py
```

Você deve ver o drone realizar, na simulação, os movimentos desejados.

## Parametros PX4 para simulação
Caso sua simulação apresente problemas no momento armar o drone ou em alguma outra etapa, veririque se os seguintes parâmetros do firmware PX4 estão com os valores a seguir: (para mais informações acesse: [Forum de discussão do erro](https://discuss.px4.io/t/offboard-mode-in-sitl/25727/3)
```
NAV_DLL_ACT = 0
NAV_RCL_ACT = 1 
COM_RCL_EXCEPT = 4
```
Os significados desses parâmetros podem ser encontrados na tabela do [PX4 User Guide/Tabela](https://docs.px4.io/main/en/advanced_config/parameter_reference.html)

Para alterar esses parâmetros do firmware use o [QgroundControl](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html) e siga as instruções descritas na [PX4 User Guide/advanced_config/parameters](https://docs.px4.io/main/en/advanced_config/parameters.html).

## Troubleshoot VM da disciplina

### O Comando catkin build falha e o worspace que aparece no relatório do comando não é o workspace indicado por você

Ao usar o comando ```catkin build -j2``` o sistema tenta dar build na px4 e ou em um workspace que não foi aquele designado por você.

Solução: tente rodar
```
cd  #go to home dir
ls -la | grep catkin  #cheak if there is a dir named *.catkin_tools*
```
veja se há arquivos .catkin_tools após rodar esse comando. Se sim, isso é o problema. Utilize o comando a seguir para remove-lo.
```
rm -r .catkin_tools
```

## Exercício para aprofundamento do aprendizado
* Faça um algorítimo para que o dorne realize uma trajetória no formato de um quadrado
* Altere a sintonia dos controladores PID no firmware PX4 para eliminar o overshoot, isto é, imponha regime supercrítico (preferencialmente crítico)
* Bonus: Imponha uma trajetoria polinomial para que o drone passe em cada vértice com velocidade não nula. Referência: Robótica, 3ª Edição
John J. Craig - Capitulo 7: Geração de Trajetórias.

## Referências

1. [Site da PX4](https://px4.io/)
2. [PX4 User Guide](https://docs.px4.io/master/en/)
3. [MAVLink Developer Guide](https://mavlink.io/en/)
4. [MAVROS](http://wiki.ros.org/mavros)
5. [Gazebo](http://gazebosim.org/)
