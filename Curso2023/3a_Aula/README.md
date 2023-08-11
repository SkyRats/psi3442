# Prática de ROS (Robotic Operating System) com TurtleSim 

Nesssa atividade iremos:
* criar um workspace do ROS;
* criar um packge ROS de controle de tartarugas do TurtleSim (Simulador de tartarugas);
* enteder como programar um script em python para controlar um robô (tartaruga) via grafo do ROS.


## 1 Criando ROS Workspace

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

```
```

```
```

## 3 Explorando o TurtleSim
## 4 Explorando ROS
### 4.1 Nodes
### 4.2 Topics
### 4.3 Messages
### 4.4 Services
### 4.5 Consulta de Tópicos/Services na ROSwiki
## 5 Estratégia de Programação
## 6 Problema
## 7 Solução via comunicação Assíncrona
### 7.1 Programa em Python
### 7.2 Execução do programa
## 8 Solução via comunicação Síncrona
### 8.1 Programa em Python
### 8.2 Execução do programa
