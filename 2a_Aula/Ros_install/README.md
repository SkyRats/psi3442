# Instalação do ROS/Gazebo
Esse tutorial de instalção do ROS foi baseado no seguinte [link](http://wiki.ros.org/ROS/Installation). O tutorial abaixo foi feito para o sistema operacional Ubuntu.

Atualmente há varias versões do ROS, porém, como versões muito recentes ainda não são muito estáveis com alguns softwares que são requisitos da diciplina, recomendamos a utilização do ROS **Melodic**. Ainda assim, é possível utilizar outras versões do ROS, como a Noetic e Kinetic. Para isso, basta substituir nas instalações o distro do ROS (melodic) pela versão que deseja instalar.

## 1. Instalação
### 1.1 Pré-instalação
Antes de mais nada você deve configurar seu repositório do Ubuntu para permitir repositórios "restricted," "universe," e "multiverse.". Normalmente o Ubuntu já vem com essas configurações certas para a instalção, mas caso deseje conferir, você pode fazer isso com [esse tutorial](https://help.ubuntu.com/community/Repositories/Ubuntu).
### 1.2 Configurando o seu source.list
Para que seu computador aceite pacotes do servidor da ROS, é necessário configuar o source.list. Para isso, basta abrir um terminal (`Ctrl+Alt+T`) e rodar o seguinte codigo:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 
```
Caso queira entender mais a finalidade dessa etapa, veja esse [tutorial](https://linuxhint.com/sources_list_ubuntu/).
### 1.3 Configurar a chave de acesso
```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
### 1.4 Instalando
Verifique primeiro se sua lista de pacotes Debian estão atualizados:
```bash
sudo apt update
```

Agora instale a versão completa do ROS, que já vem com Gazebo, Rviz e Rqt:
```bash
sudo apt install ros-melodic-desktop-full
```
## 2. Configurando Ambiente
Para utilizar as variáveis do ambiente ROS você possui duas opções:
1. Adicionar um script ao .bashrc , para que toda vez que voce inicializa um terminal ele já venha com as variaveis do ROS:
```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```
2. Rodar o script manualmente toda vez que abre o terminal:
```bash
source /opt/ros/melodic/setup.bash
```
### 2.1 Instalando dependencias para criação de pacotes
Para instalar ferramentas e dependencias de para a criação de pacotes ROS:
```bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```
Outra ferramenta muito útil na criação de pacotes é a `catkin_tools` , que pode ser instalada com ajuda desse [link](https://catkin-tools.readthedocs.io/en/latest/installing.html).
#### 2.1.1 Inicializando rosdep
Para conseguir usar muitas ferramentas do ROS, é necessário primeiro inicializar o rosdep,.
Se ainda não possue rosdep, rode:
```bash 
sudo apt install python-rosdep
```
E para inicializa-lo:
```bash
sudo rosdep init
rosdep update
```
## Pronto, o seu ROS já deve estar devidamente funcionando
Para testa-lo, abra um terminal e digite:
```bash
roscore
```
A saida deve ser algo do tipo:
```
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://user:36869/
ros_comm version 1.14.7


SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.7

NODES

auto-starting new master
process[master]: started with pid [11584]
ROS_MASTER_URI=http://user:11311/

setting /run_id to 586a22fa-e599-11ea-9764-6432a8835815
process[rosout-1]: started with pid [11596]
started core service [/rosout]
