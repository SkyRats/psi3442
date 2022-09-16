# Tutorial de Utilização de ROS (Robotic Operating System)

O objetivo desse tutorial é se familiarizar com o ambiente ROS por meio de um exemplo simples que se inicie na criação de um workspace
até um script pyhton que controle um robo utilizando a estrutura do ROS.  

Alguns comandos adicionais serão passados a fim de tornar esse tutorial útil para qualquer pessoa que deseje reproduzir os passos aqui indicados
por utilizando a SBC Labrador do CITI-USP com Debian 10 e ROS Noetic ou versões similares dessas tecnologias.

## Indice do tutorial
* Criando um workspace
* Criando um packge
* Criando um controlador em python usando ROS
* Executando a simulação do TurtleSim e do controle.py

## Criando um workspace
Criar um workspace nada mais é do que criar um diretório preparado para aplicações baseadas em ROS. 

Instalações nescessárias:
```
sudo apt install python3-catkin-tools python3-osrf-pycommon
sudo apt-get update && sudo apt-get install build-essential
```

Realize os seguintes comandos no teminal para criar o workspace:
```
mkdir -p workshop21_workspace/src
cd ~/workshop21_workspace/src
catkin build
```

Para que seu workspace possa ser permanentemente reconhecível pelo ROS, edite o bashrc fazendo:
```
nano ~/.bashrc
```
Na última linha do script do seu bashrc inclua:
```
source ~/workshop21_workspace/devel/setup.bash
```
Utilize ctrl+x para sair do ambiente de edição do nano. Salve as alterações.


## Criando um packge
Realize o s seguintes comandos no terminal.

Instale:
```
sudo apt-get install protobuf-compiler
cd workshop21_workspace
rm -rf build/ devel/
```

```
cd ~/ workshop21_workspace/src
catkin_create_pkg turtle_control_demo1 rospy roscpp
cd ~/workshop21_workspace/src/turtle_control_demo1
mkdir scripts
```
Criei um arquivo **.py**, sugestão "turtle_control_demo1.py" na pasta scripts
Observação importante: ROS noetic é preparado para rodar com python 3. Certifique-se de ter essa versão de python
e de seu código chamar recursos somnente de pyhton3.

Em seguida execute
```
cd ~/workshop21_workspace/src/turtle_control_demo1/scripts
chmod + x turtle_control_demo1.py
```

Para rebuild o workspace:
```
cd ~/workshop21_workspace
catkin build
```

## Criando um controlador em python usando ROS

## Executando a simulação do TurtleSim e do controle.py

Instalações:
```
sudo apt-get update
sudo apt-get -y install python-pip
# sudo apt-get -y install python3-pip
pip install pyyaml
```
Ações (facilitado se usar o terminator, mas exige a instalação dessa ferramenta a parte):

Abra um terminal e ligue o ROS
```
roscore
```
Abra outro terminal e ligue o turtleSim
```
rosrun turtlesim turtlesim_node
```
Abra um terceiro terminal e execute o .py criado para controlar o robo
```
rosrun turtle_control_demo1 turtle_control_demo1.py
```




