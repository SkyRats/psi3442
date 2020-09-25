# Aula 05

### Objetivo

Implementar sensores no drone dentro do ambiente de simulação Gazebo.

### Atividade 0 - Clonar o repositório com os packages necessários

Primeiramente abra o seu Terminal e vá até o diretório **src** do seu **Workspace**

Se você seguiu os passos de instalação que se encontram na documentação oficial da MAVROS, provavelmente o seu workspace se chama "catkin_ws". Se esse for o seu caso, rode o seguinte comando no Terminal:

```bash
cd catkin_ws/src
```

clone o reposótório **Simulation** que se encontra no *GitHub* da Skyrats.

```bash
git clone https://github.com/SkyRats/simulation
```

Agora volte na raiz do workspace e compile-o com `catkin_build`:

```bash
cd  ..
```

e depois 

```bash
catkin build
```

## Atividade 1 - Iniciar Simulação

Para iniciar a simulação teríamos que rodar alguns comando para setar varáveis e endereços, e rodar comandos dos ros. Porém nós já criamos um script que automatiza isso tudo e nos facilita a focar no objetivo principal de hoje.

Para rodar o script de simulação, vá até o diretório de *simulation* e depis entre em *scripts*

```bash
cd ~/catkin_ws/src/simulation/scripts
```

Agora rode o script em bash

```bash
rosrun simulate simulate.sh
```

Nesta aula nós não iremos utilizar todos os arquivos .launch disponíveis nesse package, portanto siga os seguintes passos:

- Digite 1 para escolher a opção do script onde ele te pergunta qual launch file você deseja rodar
- Digite 7 para escolher o arquivo "irirs_fpv_cam.launch"

O seu terminal deve estar asism o o da imagem abaixo:

![Terminal simulate.sh](images/terminal_1.jpeg)