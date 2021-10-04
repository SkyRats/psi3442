# Exercício em Aula

O objetivo dessa aula é aprofundar os conhecimentos adquiridos durante o curso, colocando em prática por meio da construção de um controle do drone offboard com a MAVROS.

## Pré-requisitos

Antes de começar, vocês precisam ter instalados no seu computador ou na máquina virtual:

- Firmware PX4
- QGroundControl
- ROS
- Gazebo
- MAVROS

Na máquina virtual, esses softwares já estão instalados.
Caso não tenha conseguido, olhe os guias de instalação e chamem os monitores.

## Atividade 1 - Simulação

Nessa atividade vamos inciar uma simulação no gazebo e executar um takeoff, dando o comando no terminal da PX4.

**1. Iniciar a simulação**

Vamos abrir uma simulação de um drone em um mundo vazio. Para isso, abra o terminal e rode os comandos a seguir:

```bash
cd src/Firmware # é necessário estar no diretório do Firmware para rodar
make px4_sitl gazebo
```

**2. Armar o drone**

Depois que a simulação tenha sido aberta, vamos armar o drone. Para isso, vamos rodar um comando no terminal da PX4 (basta dar um enter no terminal que está rodando a simulação):

> `commander arm`

**3. Dar takeoff**

Agora que o drone está armado, podemos levantar voo. No mesmo terminal, rode o comando:

> `commander takeoff`

O drone sairá do chão e em seguida entrará em safe mode. Isso acontece porque a PX4 considera que a conexão foi perdida, ja que ela recebeu o comando e depois perdeu a conexão (não foi enviado mais nenhum).

Neste caso, o safe mode está configurado para subir até uma determinada altura, retornar a posição de origem e, por fim, aterrissar.

## Atividade 2 - Workspace e Package

Vamos colocar em prática a criação de um workspace e de um package.

**1. Workspace**

Abra o terminal e rode os seguintes comandos:

```bash
mkdir -p ~/mavros_ws/src

cd mavros_ws/src
catkin_init_workspace

cd ~/mavros_ws
catkin build

source ~/mavros_ws/devel/setup.bash

# para adicionar o comando no .bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

```

Com isso criamos, o workspace que vamos usar no momento. Se você observar, durante todas as instalações que foram feitas até agora, existe um workspace chamado ```catkin_ws```.

**2. Package**

Agora vamos criar nosso package para armazenar nossos códigos.

```bash
cd ~/mavros_ws/src
catkin_create_pkg controle roscpp
```

No diretório ```src```, foi criado um novo diretório ```controle``` e nele encontramos todas as diretrizes de compilação dos códigos que terão nesse package.

## Atividade 3 - Offboard

Para realizarmos as missões vamos utilizar a MAVROS com o modo de voo offboard. Para tanto, devemos analisar o código presente no site da PX4:

> https://dev.px4.io/v1.9.0/en/ros/mavros_offboard.html

Utilizaremos ele para fazer nosso drone voar.

## Atividade 4 - Exercício Quadrado

Sabemos como fazer nosso drone decolar, mas como fazer ele se mover?

A ideia é a mesma que usada no tutorial da PX4 na decolagem. Entretanto, devemos acrescentar as posições de desejamos ir.

Com isso, façam com que o drone conclua uma trajetória que corresponda aos lados de um quadrado.

## Atividade 5 - Desafio

Como desafio, faça com que o drone se movimentando apenas "para frente", ou seja, ele deverá rotacionar a cada curva.

Para isso, precisamos conhecer melhor a MAVROS:

> http://wiki.ros.org/mavros

Dica: veja a documentação do tópico que já estamos publicando.

## Referências

* [Gazebo with ROS Melodic - PX4 Dev Guide](https://dev.px4.io/v1.9.0/en/setup/dev_env_linux.html#ros)
* [PX4 Simulation](https://dev.px4.io/master/en/simulation/)
