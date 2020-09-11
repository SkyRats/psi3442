# Exercício em Aula

- Refinar o conhecimento de pubs e subs do ROS
- Aprender os comandos básicos da px4
- Aprender controle do drone offboard com a mavros
- Controlar o drone por posição de way-point (gps disponível)
- **plus** noções de rotação do drone (quaternions)

## Atividade 0 - Instalações e Inicilização do Gazebo

Antes de começar, precisamos que vocês já tenham algumas coisas instaladas:

- Firmware PX4
- QGroundControl
- ROS
- Gazebo
- MAVROS

Se você ainda não conseguiu compilar o Firmware e rodar a simulação no Gazebo, siga estes passos:

**1. Abra um terminal e insira:**

```bash
nano ubuntu_sim.sh
```

Isso vai abrir um editor de texto (o Nano) no seu terminal.

**2. Cole o seguinte código:**

```bash
#!/bin/bash

## Bash script for setting up a PX4 development environment for Pixhawk/NuttX targets on Ubuntu LTS (16.04).
## It can be used for installing simulators and the NuttX toolchain.
##
## Installs:
## - Common dependencies libraries, tools, and Gazebo8 simulator as defined in `ubuntu_sim.sh`
## - NuttX toolchain (i.e. gcc compiler)

echo "Downloading dependent script 'ubuntu_sim.sh'"
# Source the ubuntu_sim.sh script directly from github
ubuntu_sim=$(wget https://raw.githubusercontent.com/PX4/Devguide/v1.9.0/build_scripts/ubuntu_sim.sh -O -)
wget_return_code=$?
# If there was an error downloading the dependent script, we must warn the user and exit at this point.
if [[ $wget_return_code -ne 0 ]]; then echo "Error downloading 'ubuntu_sim.sh'. Sorry but I cannot proceed further :("; exit 1; fi
# Otherwise source the downloaded script.
. <(echo "${ubuntu_sim}")

# NuttX
sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo \
    libftdi-dev libtool zlib1g-dev -y

# Clean up old GCC
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded -y
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa -y


# GNU Arm Embedded Toolchain: 7-2017-q4-major December 18, 2017
gcc_dir=$HOME/gcc-arm-none-eabi-7-2017-q4-major
echo "Installing GCC to: $gcc_dir"
if [ -d "$gcc_dir" ]
then
    echo " GCC already installed."
else
    pushd .
    cd ~    
    wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
    tar -jxf gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
    exportline="export PATH=$HOME/gcc-arm-none-eabi-7-2017-q4-major/bin:\$PATH"
    if grep -Fxq "$exportline" ~/.profile; then echo " GCC path already set." ; else echo $exportline >> ~/.profile; fi
    . ~/.profile
    popd
fi

#!/bin/bash

## Bash script for setting up a PX4 development environment on Ubuntu LTS (16.04).
## It can be used for installing simulators (only) or for installing the preconditions for Snapdragon Flight or Raspberry Pi.
##
## Installs:
## - Common dependencies and tools for all targets (including: Ninja build system, Qt Creator, pyulog)
## - FastRTPS and FastCDR
## - jMAVSim simulator dependencies
## - PX4/Firmware source (to ~/src/Firmware/)

# Preventing sudo timeout https://serverfault.com/a/833888
trap "exit" INT TERM; trap "kill 0" EXIT; sudo -v || exit $?; sleep 1; while true; do sleep 60; sudo -nv; done 2>/dev/null &

# Ubuntu Config
echo "We must first remove modemmanager"
sudo apt-get remove modemmanager -y


# Common dependencies
echo "Installing common dependencies"
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake build-essential genromfs ninja-build exiftool astyle -y
# make sure xxd is installed, dedicated xxd package since Ubuntu 18.04 but was squashed into vim-common before
which xxd || sudo apt install xxd -y || sudo apt-get install vim-common --no-install-recommends -y
# Required python packages
sudo apt-get install python-argparse python-empy python-toml python-numpy python-dev python-pip -y
sudo -H pip install --upgrade pip
sudo -H pip install pandas jinja2 pyserial pyyaml
# optional python tools
sudo -H pip install pyulog

# Install FastRTPS 1.7.1 and FastCDR-1.0.8
fastrtps_dir=$HOME/eProsima_FastRTPS-1.7.1-Linux
echo "Installing FastRTPS to: $fastrtps_dir"
if [ -d "$fastrtps_dir" ]
then
    echo " FastRTPS already installed."
else
    pushd .
    cd ~
    wget https://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-7-1/eprosima_fastrtps-1-7-1-linux-tar-gz -O eprosima_fastrtps-1-7-1-linux.tar.gz
    tar -xzf eprosima_fastrtps-1-7-1-linux.tar.gz eProsima_FastRTPS-1.7.1-Linux/
    tar -xzf eprosima_fastrtps-1-7-1-linux.tar.gz requiredcomponents
    tar -xzf requiredcomponents/eProsima_FastCDR-1.0.8-Linux.tar.gz
    cpucores=$(( $(lscpu | grep Core.*per.*socket | awk -F: '{print $2}') * $(lscpu | grep Socket\(s\) | awk -F: '{print $2}') ))
    (cd eProsima_FastCDR-1.0.8-Linux && ./configure --libdir=/usr/lib && make -j$cpucores && sudo make install)
    (cd eProsima_FastRTPS-1.7.1-Linux && ./configure --libdir=/usr/lib && make -j$cpucores && sudo make install)
    rm -rf requiredcomponents eprosima_fastrtps-1-7-1-linux.tar.gz
    popd
fi

# jMAVSim simulator dependencies
echo "Installing jMAVSim simulator dependencies"
sudo apt-get install ant openjdk-8-jdk openjdk-8-jre -y

# Clone PX4/Firmware
clone_dir=~/src
echo "Cloning PX4 to: $clone_dir."
if [ -d "$clone_dir" ]
then
    echo " Firmware already cloned."
else
    mkdir -p $clone_dir
    cd $clone_dir
    git clone https://github.com/PX4/Firmware.git
fi


# Go to the firmware directory
cd $clone_dir/Firmware

#Reboot the computer (required before building)
echo RESTART YOUR COMPUTER to complete installation of PX4 development toolchain

```

**3. Aperte *Ctrl+X* e *Enter* em seguida:**

```bash
source ubuntu_sim.sh
```

**4. Reinicie o seu computador**

Obs: Você pode fazer isso com o seguinte comando no temrinal:

```bash
sudo reboot
```

**5. Inicie a simulação via MAVROS**

```
make px4_sitl_default gazebo_iris_opt_flow
```


</br>

## Atividade 1

Com uma simulção aberta rodando com o PX4 rode o comando a seguir no terminal da PX4

> `commander arm`

Verifica-se que o drone agora esta armado. Rode então o comando

>  `commander takeoff`

O drone sairá do chão e em seguida entrará em safe mode, primeiro irá subir até uma altura e depois retornará para a posição de origem e irá aterrisar. Isso acontece porque a PX4 considera que a conexão foi perdida, ja que ela recebeu apenas um comando e depois perdeu a conexão.

Para realizarmos as missões vamos utilizar a mavros com o modo de voo offboard. Para tanto devemos analisar o código presente no site da px4

> https://dev.px4.io/v1.9.0/en/ros/mavros_offboard.html

Utilizaremos ele para fazer nosso drone voar pela primeira vez.


## Atividade 2

Sabemos como fazer nosso drone decolar, mas como fazer ele se mover, a ideia é a mesma que usada no tutorial da px4 para subir, mas acrescentar as posições que desajamos ir, façam um quadrado.

## Atividade 3

Vamos descobrir agora como girar o drone, mas para isso precisamos conhecer melhor a mavros

> http://wiki.ros.org/mavros

Dica olhem o tópico que ja estamos publicando e veja sua documentação.

## Referências

* [Gazebo with ROS Melodic - PX4 Dev Guide](https://dev.px4.io/v1.9.0/en/setup/dev_env_linux.html#ros)
* [PX4 Simulation](https://dev.px4.io/v1.9.0/en/simulation/)