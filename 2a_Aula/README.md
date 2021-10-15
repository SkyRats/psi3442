# ROS - Robot Operating System

## O que queremos fazer?

**Digamos que nós queremos construir um sistema autônomo**. Vamos precisar de algorítmos de controle, integração de sensores, máquinas de estados, softwares de segurança e muitos outros...  Construir todo o código para cada um desses sistemas (sem falar na integração entre eles) seria muito complicado.

Para projetos maiores e mais complexos, **é quase impossível e bastante desnecessário implementar tudo do zero**. Muitos dos problemas já têm soluções disponíveis publicamente, e podemos investir nosso tempo aprimorando soluções novas. Ter um sistema que nos permite facilmente integrar funcionalidades nossas e de outros desenvolvedores seria muito legal...

**E é exatamente isso que o ROS faz**

## O que é o ROS afinal?

> Robot Operating System (ROS, sistema operacional de robôs) é uma coleção de frameworks de software para desenvolvimento de robôs, que fornece a funcionalidade de um sistema operacional em um cluster de computadores heterogêneo. ROS fornece serviços padrões de sistema operacional, tais como abstração de hardware, controle de dispositivos de baixo nível, a implementação de funcionalidades comumente usadas, passagem de mensagens entre processos e gerenciamento de pacotes.

Ou seja, o ROS é um ecossistema de softwares e ferramentas padronizadas que nos permite **integrar os diversos componentes** necessários para o funcionamento de um robô.

*Alguns recursos interessantes para procurar mais informações sobre o ROS e suas funcionalidades são a [ROS Wiki](http://wiki.ros.org/) e o [ROS Answers](https://answers.ros.org/questions/)*

## Como a Skyrats usa o ROS?

Usamos o ROS conforme as necessidades do projeto em que estamos trabalhando. Podemos mudar os pacotes utilizados, mas a estrutura por baixo permanece sempre a mesma. Alguns pacotes que usamos são:

* MAVROS
* Gazebo

## Como funciona o ROS?

O funcionamento do ROS é representado por um **Grafo de comunicação**, que organiza a interação entre os programas através de tópicos e serviços.

![grafo](images/grafo.png)

Grafos são estruturas muito utilizadas na computação e na matemática, e consistem em um conjunto de **vértices** ou **nós** ligados por **arestas**. Na prática, eles funcionam como uma forma de abstração, isto é, uma forma de compreender e modelar um sistema. No caso do ROS, o grafo representa a comunicação: os nós são os programas, e as arestas representam vias de comunicação entre eles.

Para iniciar o ROS no nosso computador, usamos o comando `roscore` em um terminal.

### Nodes

Os programas (em C++, Python ou Lisp) que participam na execução do ROS são chamdos de ***Nodes*** (ou Nós). Na verdade, qualquer programa que queira interagir com outros _nodes_ ou usar uma funcionalidade do ROS **é obrigado a se declarar como um node** no código (veremos como fazer isso mais para a frente).

Para ver os _nodes_ rodando no momento, executamos o seguinte comando em um outro terminal:

```
rosnode list
```

Note que, no momento, temos apenas um _node_ em funcionamento, o `/rosout`. Para executar um programa (que seja um _node_), utilizamos o comando `rosrun`. Vamos começar executando o turtlesim, o simulador de tartarugas. Ele está contido no package _turtlesim_, e o simulador em si corresponde ao programa _turtlesim_node_.

```
rosrun turtlesim turtlesim_node
```

Veja que o programa roda e abre a linda janelinha do turtlesim. Examinando novamente os _nodes_ em execução, vemos que apareceu o `/turtlesim`.

**Comandos importantes:**
- `rosnode list`: lista todos os _nodes_ ativos
- `rosnode info node_name`: dá informações sobre um _node_ específico.
- `rosrun package_name program_name`: executa um _node_ em um package.

### Topics

Mas como esses _nodes_ se comunicam entre si? A forma mais comum é através de _topics_. Os tópicos funcionam como caixinhas de correio: um _node_ coloca uma mensagem em uma certa caixinha, e depois um outro _node_ pode ir nessa caixinha e ver a mensagem que foi colocada. No ROS, dizemos que um _node_ do tipo _Publisher_ publica mensagens em um _topic_, e um _node_ do tipo _Subscriber_ se inscreve nesse _topic_, recebendo todas as mensagens que são publicadas nele.

Eles são um dos tipos possíveis de **arestas** no grafo de comunicação que mencionamos acima. Mais especificamente, são arestas direcionais, ou seja, que só permitem comunicação em um sentido. Dizemos também que a comunicação é **assíncrona**, pois os Publishers podem realizar as postagens em qualquer momento, sem precisar interromper sua execução para esperar que algum Subscriber receba a mensagem.

Vamos ver os tópicos atualmente em execução com o comando `rostopic list` em um terminal. Vamos examinar, por exemplo, o tópico `/turtle1/pose`, com o comando `rostopic info /turtle1/pose`:

```
Type: turtlesim/Pose

Publishers:
 * /turtlesim (http://henrique-Q550LF:38043/)

Subscribers: None
```

Esse tópico é publicado pelo _node_ `/turtlesim`, com mensagens do tipo `/turtlesim/Pose`. Neste tópico, o simulador publica a posição da tartaruginha turtle1. Vamos ver o que ele está publicando com o comando

```
rostopic echo /turtle1/pose
```

Uma ferramenta importante para visualizar o que está acontecendo é o _rqt_graph_. Executando `rqt_graph` em um terminal, vemos a seguinte tela:

![rqt_graph](images/rqt_graph.png)

Os _nodes_ são as elipses, os tópicos são os retângulos, e as setas indicam o fluxo de dados. Nesse caso, o /turtlesim publica mensagens no tópico /turtle1/pose, e um _node_ correspondendo ao nosso terminal com `rostopic echo` está inscrito nesse tópico.

Vamos analisar agora o tópico `/turtle1/cmd_vel`. Vemos que não há nenhum programa publicando nele, e o turtlesim é o único Subscriber. Nesse tópico, podemos publicar mensagens de velocidade, e o turtlesim irá recebê-la para fazer a tartaruga andar. O interessante disso é que nós não precisamos saber dos detalhes de como ele faz a tartaruga andar, basta publicarmos uma mensagem padrão nesse tópico e a mágica acontece.

Essa funcionalidade é implementada pelo seguinte node:

```
rosrun turtlesim turtle_teleop_key
```

Esse programa basicamente recebe comandos do seu teclado e envia uma mensagem equivalente no topico `/turtle1/cmd_vel`. Poderiamos fazer algo parecido diretamente pela linha de comando:

```
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 1.0]'
```

**Obs.:** Tópicos *não* são programas. Eles só começam a existir quando um _node_ inicia um Publisher ou Subscriber.

**Comandos importantes:**
- `rostopic list`: lista todos os tópicos
- `rostopic info /topic_name`: dá informações sobre um tópico específico
- `rostopic echo /topic_name`: imprime na tela todas as mensagens publicadas em um tópico específico
- `rostopic pub /topic_name msg_type args`: publica uma mensagem em um tópico
- `rostopic find msg_type`: encontra tópicos com um certo tipo de mensagem
- `rosmsg show msg_type`: mostra a descrição de um dado tipo de mensagem

### Messages

Em um tópico só podem ser passadas mensagens de apenas um tipo, e tentar usar uma mensagem do tipo errado resultaria em um erro. Mensagens são compostas de alguns tipos primitivos, e são descritas em arquivos .msg dentro de um package. Para serem usadas no código, o ROS transforma elas em classes.

_Ex.:_ Vimos que o tipo de mensagem usado no tópico cmd_vel é geometry_msgs/Twist. Vamos usar o comando `rosmsg show geometry_msgs/Twist` para ver o que essa mensagem contem:

```
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

Veja que ela é composta de dois itens (linear e angular), sendo que esses são instâncias de uma mensagem do tipo geometry_msgs/Vector3. Essa mensagem, por sua vez, tem 3 parâmetros (dessa vez primitivos): x, y e z, do tipo float64. Ou seja, uma mensagem pode ser uma combinação de outras mensagens e tipos primitivos.

Você pode encontrar mais informações sobre ROS Messages [aqui](http://wiki.ros.org/msg).

### Services

O modelo publish/subscribe que estávamos usando até agora é muito útil pra várias aplicações, mas possui limitações. Por exemplo, um Publisher nunca sabe se o Subscriber recebeu a mensagem, nem se ele conseguiu realizar a tarefa.

Para resolver isso, existem os _Services_. Ao invés de "postar" uma mensagem num tópico, os serviços correspondem a chamadas diretas que um node faz para o outro, da mesma forma que podemos chamar funções em Python. Os serviços são compostos de um _request_, que é a mensagem que o node "cliente" realiza para o "servidor", e uma _response_, a mensagem retornada pelo servidor ao cliente quando a chamada for concluida. É importante notar que **enquanto o servidor estiver realizando o serviço, o processamento do cliente é interrompido**. Por isso, dizemos que a comunicação por services é **síncrona**, e constitui um outro tipo de **aresta** para a comunicação entre nós. 

Por exemplo, o turtlesim possui um serviço chamado /spawn, que cria uma nova tartaruga. Vamos criar uma tartaruga chamada Edson no ponto x=1, y=2 com theta=0.

```
rosservice call /spawn 1 2 0 Edson
```

**Comandos importantes:**
- `rosservice list`: lista todos os serviços ativos
- `rosservice call /service_name [args]`: chama um serviço com os argumentos dados
- `rosservice info /service_name`: mostra informações sobre um dado serviço


### Parameter Server

O **Parameter Server** é um dicionário de parâmetros centralizado que é criado durante a execução do ROS e que pode ser usado por qualquer _node_. Ele é útil para definir nomes, opções e configurações ao executar um programa, assim como modificá-los durante a execução (sem precisar, desligar o programa, modificar o código, compilar e rodar de novo). Também é possível guardar e carregar esses parâmetros em arquivos, o que pode facilitar o transporte de configurações.

Executando o comando `rosparam list` podemos ver todos os parâmetros atualmente sendo usados. Por exemplo, o turtlesim armazena a cor de fundo do simulador em 3 parâmetros (RGB). Podemos ver qual o valor armazenado em um desses com:

```
rosparam get /sim/background_b
```

Podemos também mudar o valor do azul para 0, por exemplo:

```
rosparam set /sim/background_b 0
```

Note que o simulador não atualiza a cor de fundo em tempo real. Para isso, seria necessário chamar algum serviço. Qual deve ser esse serviço?

*Obs.:* O parâmetro só passa a "existir" quando um valor é atribuido a ele pela primeira vez.

**Comandos importantes:**
- `rosparam list`: lista todos os parâmetros do Parameter Server
- `rosparam get /param_name`: mostra o valor armazenado em um parâmetro
- `rosparam set /param_name [args]`: atribui um valor a um parâmetro, eliminando o anterior.

### Packages
Todo software ROS é organizado em **packages**. Eles são um jeito modular de organizar pedaços de código com funções diferentes, e geralmente estão presentes no nosso **catkin workspace**.
Cada package ROS é (idealmente) independente dos códigos presentes nos outros packages, e pode conter **nodes**, **messages**, **config**'s, **libraries** ou qualquer coisa que constitua um módulo útil.
> In general, ROS packages follow a "Goldilocks" principle: enough functionality to be useful, but not too much that the package is heavyweight and difficult to use from other software.

De maneira prática, um package ROS é organizado em um repositório "compilável", que contém:

* Um arquivo `CMakeLists.txt`

Nele, descrevemos as diretrizes de compilação dos códigos desse package.

* Um arquivo `package.xml`, que fornece informações básicas sobre o código como
    * Versão
    * Nome
    * Informações dos desenvolvedores
    * Licensa
    * Balela, perfumaria

* Uma pasta `src`, onde encontramos todos os codigos fontes que serão compilados

* Uma pasta `scripts`, para guardar scripts executáveis (python, bash, etc)

* Uma pasta `include/package_name`

Nela, colocamos todos os **headers C++** - os famosos .h. Para serem reconhecidos por este e por outros pacakges ROS

* Além destas, podemos ter pastas para definir ROS **messages** e **services**, denominadas
    * `msg`
    * `srv`

* Uma pasta `launch` para guardar os arquivos launchfiles

#### Criando o package
Putz, mas como eu vou lembrar de criar todas essas pastas?
Resposta: ~~não vou~~

Para a criação de um package, usamos o comando 
`catkin_create_pkg <nome do package>`

Além desse comando, temos vários outros para manipular packages, como

* `rospack` - obter informações do package
* `rosdep` - instala dependências de um package
Exemplo: `rosdep install PACKAGE` instala as dependências do package `PACKAGE` na sua máquina.


### Workspaces
Todos os packages ROS que desenvolveremos estarão organizados em **workspaces**, para organizarmos os códigos que precisamos para determinada aplicação. Esses são os **catkin workspaces**, uma pasta onde modificamos, compilamos e instalamos packages.
A carinha de um workspace...
```
workspace_folder/         -- WORKSPACE
  src/                    -- SOURCE SPACE
    CMakeLists.txt        -- The 'toplevel' CMake file
    package_1/
      CMakeLists.txt
      package.xml
      ...
    package_n/
      CMakeLists.txt
      package.xml
      ...
  build/                  -- BUILD SPACE
  devel/                  -- DEVELOPMENT SPACE (set by CATKIN_DEVEL_PREFIX)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin
    env.bash
    setup.bash
    setup.sh
    ...
  install/                -- INSTALL SPACE (set by CMAKE_INSTALL_PREFIX)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin             
    env.bash
    setup.bash
    setup.sh
```

* `src`, nosso velho amigo

Na pasta `src` que estarão todos os códigos (compiláveis) que desenvolveremos - src <= source code <= código fonte.

* `build` irá guardar todos os informações e arquivos intermediarios de compilação do cmake.

* `devel` é onde todos os resultados das compilação estão, os binários e executáveis

* `install` os programas podem ser instalados no computador no diretório install, com o comando `make install`



Para instalar as dependências de todos os packages de um workspace, podemos usar o comando `rosdep install --from-paths src --ignore-src -r -y`, rodando ele da raiz do workspace.

> "This command magically installs all the packages that the packages in your catkin workspace depend upon but are missing on your computer." - ROS Wiki

#### Criando e iniciando um workspace
Para criar um catkin workspace, precisamos ~~, além de ter o [catkin instalado](http://www.ros.org/wiki/catkin#Installing_catkin)~~ criar um diretório, que será nosso workspace, com uma pasta `src` dentro dele.

`mkdir -p ~/meu_ws/src`


Depois disso, **dentro da pasta `src`**, daremos o comando

`catkin_init_workspace`, que inicia nosso workspace (repare que ele cria o arquivo CMakeLists.txt dendo da src)


#### Como usar o workspace
Para "entrar" no workspace, usamos os chamados **environment setup files**, que são basicamente scripts bash que configuram variáveis do sistema para que o sistema reconheça o seu catkin workspace.
Esses arquivos estão em `devel/`, e o que costumamos usar (para o terminal bash) eh o `devel/bash`.
Para usar o workspace então, precisamos dar o comando

`source ~/[nome_do_workspace]/devel/setup.bash`

_Vale ressaltar que esse comando só pode ser executado após uma primeira compilação do workspace._

Costumamos deixar a linha abaixo e o source do workspace no final do nosso arquivo ~/.bashrc, para mudar o ambiente de trabalho.

`source /opt/ros/melodic/setup.bash`

#### Compilando o workspace
Para compilar, devemos ir ao diretório do workspace e dar o comando

`catkin build`

Após esse comando são criadas no workspace, que antes só possuia a pasta `src`, as pastas devel, build e logs.

E é criado também nosso amigo `devel/setup.bash`, que podemos usar para configurar o workspace no terminal bash.

*Obs.:* caso o seu terminal reclame que o comando `catkin build` não foi encontrado, é necessária a instalacao do python-catkin-tools através do comando `sudo apt-get install python-catkin-tools`.

### Implementação de um Node Publisher em Python
```python
#!/usr/bin/env python
import rospy

rospy.init_node("nome_do_node")
rate = rospy.Rate(60)
pub = rospy.Publisher("nome_do_topico", TipoDeMsg, queue_size=1)

def main():
  pub.publish(TipoDeMsg(param1, param2, ..., paramN))
  rate.sleep()
```
A primeira linha do código é chamada de **shebang**, e se temos um script executável, que é chamado para execução sem que informemos a linguagem dele (como será o caso do roslaunch/rosrun), o shebang dá o endereço do interpretador que roda o código em questão
>if the script is executable, and called without the preceding language. The script then calls the language's interpreter to run the code inside the script, and the shebang is the "guide" to find it.

### Implementação de um Node Subscriber em Python
```python
#!/usr/bin/env python
import rospy

def callback_func(data):
  print(data)

rospy.init_node("nome_do_node")
rate = rospy.Rate(60)
sub = rospy.Subscriber("nome_do_topico", TipoDeMsg, callback_func)
def main():
  rospy.spin()

```


### Chamando um Service em Python
```python
#!/usr/bin/env python
import rospy


service = rospy.ServiceProxy("service", CommandBool)

def main():
  service(arg1, arg2, ..., argN)
```

### Criando um Service em Python
```python
#!/usr/bin/env python
from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
```
Exemplo do ROS wiki

### Exemplos em C++

Ao escrever um node ROS em C++, precisaremos compilá-lo eventualmente, isso envolve adicionar o arquivo .cpp (e, eventualmente, os .h) no arquivo `CMakeLists.txt`, para fazer isso o CMakeLists facilita disponibilizando um template para cada tipo de operação. Precisamos:

* Adicionar o roscpp no `find_package`
* Adicionar o executável com um
```
add_executable([nome do executável] src/[nome_do_compilável].cpp)
```
Onde o nome do executável é de livre escolha


* Linkar as bibliotecas usadas com o target_link_libraries

```
target_link_libraries([nome_do_executável] ${catkin_LIBRARIES}, <ilb2>, <lib3>)
```

Exemplo, se quisermos adicionar o OpenCV, teremos algo do tipo
`target_link_libraries([nome do executável] ${OpenCV_LIBS})`

#### Publisher em C++
```C++
#include "ros/ros.h"
#include "std_msgs/String.h"


int main(int argc, char **argv) // argumentos passados pelo terminal
{
  ros::init(argc, argv, "publisher");
  ros::NodeHandle n; // objeto que cria os publishers, subscribers, services, etc

  ros::Publisher pub = n.advertise<std_msgs::String>("topic", 0); //topic, queue

  ros::Rate rate(10);

  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello, ROS";

    ROS_INFO("%s", msg.data.c_str());

    pub.publish(msg);
 
    ros::spinOnce();
 
    rate.sleep();
   }
   return 0;
 }
```
Exemplo do ROS wiki

#### Subscriber em C++

```C++

include "ros/ros.h"
#include "std_msgs/String.h"

void callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("%s", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("topic", 1000, callback);

  ros::spin(); // trava o programa para rodar somente o callback

  return 0;
}

```


### Roslaunch
O roslaunch é uma ferramenta que serve para **rodar diversos nodes ROS ao mesmo tempo**, e setar parametros no Parameter Server do ROS - e rodar outros arquivos launch :).
Para usá-la, criamos arquivos launch (um arquivo XML)

```XML
<?xml version="1.0"?>
<launch>

    <node pkg="your_package" name="josé" type="your_node"/>

    <arg name="x" default="0"/> <!--Seta o parametro x como 0-->
   
    <include file="$(find outropackage)/launch/file2.launch"> <!--lansa o outro arquivo-->
        <arg name="y" value="1"/> <!-- y = 1 -->
    </include>
</launch>
```

#### Rodando o roslaunch
Para rodar o roslaunh, usamos o comando

`roslaunch package_name file.launch`

Mas antes precisamos **compilar o workspace** e dar **source** em seu **setup.bash**

#### ROS Bags
Ferramenta para gravação e reprodução de mensagens ROS.

* `rosbag record [topico1] [topico2] [topico3]` grava os tópicos 1, 2 e 3
* `rosbag record -a` grava todos os tópicos
* `rosbag play [nome_da_rosbag].bag`

Exemplo: rosbag Tello

#### rviz
O rviz é uma ferramenta de visualização 3D pro ROS, onde podemos mostrar diversos tipos de mensagens ROS.


Podemos rodar o rviz com o comando

`rosrun rviz rviz`

Ou carregar uma interface já configurada com

`rosurun rviz rviz -d rviz_config.rviz`

#### Symlinks

`ln -s ~/git/[package] ~/[workspace]/src/`


## Referências
1. [ROS Wiki](http://wiki.ros.org/)
2. [Robot Operating System - Wikipedia](https://pt.wikipedia.org/wiki/Robot_Operating_System)
