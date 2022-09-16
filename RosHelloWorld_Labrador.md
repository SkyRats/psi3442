# Tutorial de Utilização de ROS (Robotic Operating System)

O objetivo desse tutorial é se familiarizar com o ambiente ROS por meio de um exemplo simples que se inicie na criação de um workspace
até um script pyhton que controle um robo utilizando a estrutura do ROS.  

Alguns comandos adicionais serão passados a fim de tornar esse tutorial útil para qualquer pessoa que deseje reproduzir os passos aqui indicados
por utilizando a SBC Labrador do CITI-USP com Debian 10 e ROS Noetic ou versões similares dessas tecnologias.

## Criando um workspace
Criar um workspace nada mais é do que criar um diretório preparado para aplicações baseadas em ROS. 

Instalações nescessárias:
```
sudo apt install python3-catkin-tools python3-osrf-pycommon
sudo apt-get update && sudo apt-get install build-essential
```

Realize os seguintes comandos para criar o workspace:
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
source ~/ workshop21_workspace / devel / setup . bash
```
Utilize ctrl+x para sair do ambiente de edição do nano. Salve as alterações.


## Criando um packge

## Criando um controlador em python usando ROS

## Executando a simulação do TurtleSim e do controle.py

