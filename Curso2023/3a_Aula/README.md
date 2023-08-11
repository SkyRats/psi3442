# Prática de ROS (Robotic Operating System) com TurtleSim 

Nesssa atividade iremos:
* criar um workspace do ROS;
* criar um packge ROS de controle de tartarugas do TurtleSim (Simulador de tartarugas);
* enteder como programar um script em python para controlar um robô (tartaruga) via grafo do ROS.


## 1 Criando ROS Workspace

O workspace é um diretório preparado para se trabalhar com o ROS em uma aplicação. Criaremos hoje um workspace para trabalhar o o turtlesim hoje. Em uma aula futura, criaremos outro worspace para trabalhar com o drone.

```
mkdir -p ros_workspaces/turtle_ws/src
cd ros_workspaces/turtle_ws/
```
```
catkin build - j2
```
```
nano ~/.bashrc
```
No nano (editor de texto sem mouse), utililze a seta para baixo e des¸ca at´e a ultima
linha e acrescente se não existir:

```
source /opt/ros/melodic/setup.bash
source ~/ros_workspaces/turtle_ws/devel/setup.bash
```
Saia do nano utilizando: ctrl + x

## 2 Criando ROS Package

O package é um diretório que contém uma funcionalidade específica. Pode ser composto por vários scritps .py e/ou c++
que em conjunto realizam uma função em uma aplicação de robótica com ROS.

O comando a seguir cria um packge chamado turtle_control habilitado para
trabalhar com C++ (roscpp) e com python (rospy).

```
cd ~/ros_workspaces/turtle_ws/src
catkin_create_pkg turtle_control rospy roscpp
```
Depois disso é preciso criar uma pasta chamada scripts caso ela não exista dentro
do pakge criado. A sequência de comandos a seguir realiza essa operação:
```
cd ~/ros_workspaces/turtle_ws/src/turtle_control
mkdir scripts
```
Crie um .py no diretório ```scripts``` chamado ```turtle_go_topic.py```
```
cd ~/ros_workspaces/turtle_ws/src/turtle_control/scripts
chmod + x turtle_go_topic.py
```
## 3 Explorando o ROS com TurtleSim

### 3.1 Consulta de Tópicos/Services via comandos no terminal

Inicie em um terminal o grafo do ROS usando Roscore para ativar o node central da rede.
```
roscore
```
Inicie em outro terminal o simulador de tartaruga, uma tartaruga aparecerá na tela
```
rosrun turtlesim turtlesim_node
```
Em outro terimnal, explore os nodes do grafo ROS disponíveis
Com isso descobrimos os topics onde o node é publisher, onde é subscriber, services disponíveis e outras informações.
```
rosnode list
rosnode info /turtlesim
```
Liste os topic disponiveis, descubra informações sobre o topic pose onde turtle1 é um node publisher nesse topic
"Escute" as mensagens que estão sendo postadas no topic. Observe que o type da message é turtlesim/Pose.
```
rostopic list
rostopic info /turtle1/pose
rostopic echo /turtle1/pose
```
Liste as messages disponíveis, descubra detalhes da message cujo tipo foi mencionada na pesquisa ```rostopic info /turtle1/pose```

```
rosmsg list
rosmsg show turtlesim/Pose
```
Liste os services disponíveis
```
rosservice list
rosservice info /turtle1/teleport_absolute
roservice call /turtle1/teleport_absolute 7 7 90
```

Exercício simples:
Utilize o service correto para criar uma tartaruga chamada Edson

### 3.1 Consulta de Tópicos/Services na ROSwiki
[RosWiki]([https://link-url-here.org](http://wiki.ros.org/turtlesim))

## 4 Estratégia de Programação

Planeje sua aplicação. Entenda o que o seu código irá fazer e faça uma decomposição funcional do problema de modo
que cada parte mais simples do código resolva um subproblema de maneira que o script completo solucione o problema.

Vamos entender isso por meio de um exemplo. Depois, abstraia o exemplo a seguir para sua aplicação, seja drone ou o que for.

Queremos controlar a posição tartaruga turtle1 do simulador turtlesim (node turtlesim). 

Como se controla posição? Alterando a velocdiade de um objeto ao longo do tempo. Para ir de (0,0) para (1,0) um objeto deve sair do repouso em (0,0), acelerar em direção ao alvo e desacelerar quando estiver nas proximidades do alvo (1,0)

Traduzindo:

controla posição? -> precisamos saber a posição do turtle1
Alterando a velocdiade -> precisamos alterar a velocidade do turtle1

posição do turtle1 -> liste os topic e procure um cujo nome remeta a posição do turtle1
velocidade do turtle1 -> liste os topic e procure um cujo nome remeta a velocidade do turtle1

verifique se o turtle1 é um publisher no topic de posição e um subscriber no topic de velocidade que você encontrou.
Se sim, prossiga. Se não, procure outros topic pois do contrário você não poderá receber a posição do turtle1 e nem
comandar usa velocidade.

Feito isso, entenda como são as messages dos topic que você encontrou e os tipos e subtipos das messages.

Com isso, agora é possível importar da biblioteca rospy as funcionalidades que permitirão implementar
um script python que se comunique com o node turtlesim através do ROS.

```
import rospy
from <tipo da mensagem>. msg import <subtipo da mensagem>
```
Por exemplo 
```
from geometry_msgs . msg import Twist
```


## 5 Problema
Escreva um script no arquivo ```turtle_go_topic.py``` criando uma rotina que quando chamada através do comando 

```
rosrun turtle_control turtle_go_topic.py
```

Seu programa deve perguntar no terminal a um usuário coordenadas desejadas (x,y) de modo que a tartagura trutle1 vá para essa posição usando a comunicação assíncrona via topics.

Ligue o seu sistema robótico ao ROS;
Consulte em quais topic o seu robo publica informações;
Consulte em quais topic o seu robo está inscrito para receber informações;

## 6 Solução via comunicação Assíncrona
### 6.1 Programa em Python
```
```

```
```

```
```

```
```
### 7.2 Execução do programa
## 8 Solução via comunicação Síncrona
### 8.1 Programa em Python
### 8.2 Execução do programa
