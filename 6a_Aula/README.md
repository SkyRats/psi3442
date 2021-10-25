# P1 PSI3442

Esse repositório é o pacote ROS que deve ser utilizado para as questões práticas da primeira prova de PSI3442 - Projeto de Sistemas Embarcados de 2021.

## Configuração inicial

O repositório funciona como um pacote ROS, logo deve ser baixado na pasta `src` do seu catkin workspace, de formaa que fique, por exemplo, em `~/catkin_ws/src/p1_psi3442`.

```bash
cd ~/catkin_ws/src/p1_psi3442 # mudar conforme caminho do seu workspace
git clone https://github.com/SkyRats/p1_psi3442.git
```

## Utilizando a simulação

Para abrir a simulação, você deve executar os seguintes códigos **dentro da pasta do pacote**.

```bash
cd ~/catkin_ws/src/p1_psi3442

# Esse comando deve ser executado em cada terminal novo
source scripts/setup.bash

# Para abrir a simulação
roslaunch p1_psi3442 simulate.launch

# Para testar: use um comando de decolagem
# No próprio terminal em que executou o comando acima
commander takeoff
```

## Executando código

Cada questão que possui código deve ser resolvida em um único arquivo. O código pode ser feito em C++ (nos arquivos `src/q2_A.cpp`, `src/q2_B.cpp` e `src/q3.cpp`) ou em Python (nos arquivos `scripts/q2_A.py`, `scripts/q2_B.py` e `scripts/q3.py`).

**Atenção**: depois de modificar os códigos em C++, é necessário recompilar
```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

Para executar um programa, basta ter a simulação aberta e utilizar o comando `rosrun`.

```bash
# Use o comando correspondente à lingua utilizada
rosrun p1_psi3442 q2.py # Python
rosrun p1_psi3442 q2 # C++
```