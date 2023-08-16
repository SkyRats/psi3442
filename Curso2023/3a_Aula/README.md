# Prática de ROS (Robotic Operating System) com TurtleSim 

Nesssa atividade iremos:
* criar um workspace do ROS;
* criar um packge ROS de controle de tartarugas do TurtleSim (Simulador de tartarugas);
* enteder como programar um script em python para controlar um robô (tartaruga) via grafo do ROS.

Dicas de linux
Abra o terminal 
```
ctrl+alt+t
```
Criar uma nova aba
```
ctrl+shift+t
```
Mudar de diretório
```
cd nome_do_diretorio/nome_do_sub_diretorio/nome_do_sub_sub_diretorio
```
Ver o que há em um diretório
```
ls
```
Prefira usar ```tab``` para completar os comandos, assim você digita mais rápido e certo.


## 1 Criando ROS Workspace

O workspace é um diretório preparado para se trabalhar com o ROS em uma aplicação. Criaremos hoje um workspace para trabalhar o o turtlesim hoje. Em uma aula futura, criaremos outro worspace para trabalhar com o drone.

No terminal
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
Crie um .py no diretório ```scripts``` chamado ```turtle_go_topic.py``` e outro chamado ```turtle_go_service.py```

```
touch turtle_go_topic.py
touch turtle_go_service.py
```
Em seguida, habilite os .py para serem executáveis
```
cd ~/ros_workspaces/turtle_ws/src/turtle_control/scripts
chmod +x turtle_go_topic.py
chmod +x turtle_go_service.py
```

Finalmente, habilite o package. Sem o comando ```source devel/setup.bash``` seu package não será encontrado pelo comando ```rosrun```. Uma dica, use ```roscd nome_do_package``` para mudar diretamente para o diretório do package a partir de qualquer diretório.

Para habilitar o package 

```
cd ~/ros_workspaces/turtle_ws/
source devel/setup.bash
```

Teste se funcionou. O comando a seguir deve mudar o seu diretório para o caminho do package ```turtle_control``` sem retornar erros.
```
roscd turtle_control
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
rosservice call /turtle1/teleport_absolute 7 7 90
```

Exercício simples:
Utilize o service correto para criar uma tartaruga chamada Edson

### 3.1 Consulta de Tópicos/Services na ROSwiki
[RosWiki]([https://link-url-here.org](http://wiki.ros.org/turtlesim))

## 4 Como usar as ferramentas do ROS no python

Planeje sua aplicação. Entenda o que o seu código irá fazer e faça uma decomposição funcional do problema de modo
que cada parte mais simples do código resolva um subproblema de maneira que o script completo solucione o problema.

Se o problema for complexo e você desejar resolvê-lo utilizando o ROS siga o procedimento:
* Ligue o seu sistema robótico ao ROS;
* Consulte em quais topic o seu robo publica informações;
* Consulte em quais topic o seu robo está inscrito para receber informações;
* Consulte os services disponíveis;
* Elabore uma estratégia de como usar os topic/services disponíveis para operar o seu robô;
* Sintetize seu projeto por meio scripts de comando;
* Valide por simulações/testes práticos simples.

Vamos entender isso por meio de um exemplo. Depois, abstraia o exemplo a seguir para sua aplicação, seja drone ou o que for.

Exemplo: controlar a posição tartaruga turtle1 do simulador turtlesim (node turtlesim). 

Como se controla posição? Alterando a velocdiade de um objeto ao longo do tempo. Para ir de (0,0) para (1,0) um objeto deve sair do repouso em (0,0), acelerar em direção ao alvo e desacelerar quando estiver nas proximidades do alvo (1,0)

Traduzindo:

controla posição? -> precisamos saber a posição do turtle1
Alterando a velocdiade -> precisamos alterar a velocidade do turtle1

### 4.1 Comunicação Assíncrona

posição do turtle1 -> liste os topic e procure um cujo nome remeta a posição do turtle1
velocidade do turtle1 -> liste os topic e procure um cujo nome remeta a velocidade do turtle1

Terminal 1
```
roscore
```
Terminal 2
```
rosrun turtlesim turtlesim_node
```
Terminal 3
```
USE OS COMANDOS DA SEÇÃO 3 PARA PESQUISAR TOPICS, MESSAGES, PUBLISHERS E SUBSCRIBERS
```

verifique se o turtle1 é um publisher no topic de posição e um subscriber no topic de velocidade que você encontrou.
Se sim, prossiga. Se não, procure outros topic pois do contrário você não poderá receber a posição do turtle1 e nem
comandar usa velocidade.

Feito isso, entenda como são as messages dos topic que você encontrou e os tipos e subtipos das messages.

Note que o comando
```
rostopic info /turtle1/pose
```
retorna no terminal
```
aluno@aluno-VirtualBox:~$ rostopic info /turtle1/pose
Type: turtlesim/Pose

Publishers:
 * /turtlesim (http://aluno-VirtualBox:46137/)

Subscribers: None

```
De onde vemos que o tipo (type) da mensagem é turtlesim
E o subtipo da mensagem é Pose

Com isso, agora é possível importar da biblioteca rospy as funcionalidades que permitirão implementar
um script python que se comunique com o node turtlesim através do ROS.

```
import rospy
from <tipo da mensagem>.msg import <subtipo da mensagem>
```
Por exemplo 
```
from turtlesim.msg import Pose
```

Agora vamos criar um script simples que é capaz de ler as informações do topic /turtle1/pose
Para ver se o script funciona, iremos mostrar o resultado da leitura na tela.

### Observação Importante: Shebang line

Esses comandos que aparecem no início dos script python são chamados shebang line. Você pode usar ```#!/usr/bin/env``` de modo que o interpretador usado será o primeiro no seu ambiente do ```$PATH```. Ou, se desejar especificar uma versão específica do python utilize:

```
#!/usr/bin/env python
```
ou
```
#!/usr/bin/env python3
```

Em um terminal linux, use o comando a seguir para verificar sua versão do python.  
```
python -V
```

### Código em python

```python
#!/usr/bin/env python

#Libraries
import rospy
from turtlesim.msg import Pose

class Turtle:
    def __init__(self): #Calss constructor
        self.pose = Pose() #turtle's pose atribute 
        rospy.init_node("readPose") #start a node called "readPose" inteh ROS' graph
        self.rate = rospy.Rate(10) # Set the update rate of 10Hz
        #define "readPose" node as subscriber of /turtle1/pose topic fed by turtlesim node.
        self.pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback) 

    #def a calback function for the subscriber knows how to get the "data"
    def pose_callback(self,data):
        self.pose.x = data.x #m
        self.pose.y = data.y #m
        self.pose.theta = data.theta #degerees

    #alternative callback getting (x,y) coordinates in cm
    def pose_callback_cm(self,data):
        self.pose.x = 100*data.x #cm
        self.pose.y = 100*data.y #cm
        self.pose.theta = data.theta #degerees

    #method to show position in terminal
    def show_pose(self):
        while not rospy.is_shutdown():
            print("(x,y,theta)=("str(self.pose.x)+","+str(self.pose.y)+","+str(self.pose.theta)+")")
            self.rate.sleep()

if __name__ == '__main__':
    turtle = Turtle()
    turtle.show_pose()

```

#### 4.1.1 Vizualização gráfica via rqt_graph

Uma ferramenta importante para visualizar o que está acontecendo é o _rqt_graph_. Executando `rqt_graph` em um terminal, vemos a seguinte tela:

![rqt_graph](images/rqt_graph.png)

Os _nodes_ são as elipses, os tópicos são os retângulos, e as setas indicam o fluxo de dados. Nesse caso, o /turtlesim publica mensagens no tópico /turtle1/pose, e um _node_ correspondendo ao nosso terminal com `rostopic echo` está inscrito nesse tópico.

Terminal 1
```
roscore
```
Terminal 2
```
rosrun turtlesim turtlesim_node
```
Terminal 3
```
rostopic echo /turtle1/pose
```
Terminal 4
```
rqt_graph
```

### 4.2 Comunicação Sincrona

posição do turtle1 -> liste os service do node turtlesim e procure um cujo nome remeta a alteração de posição do turtle1

```
aluno@aluno-VirtualBox:~$ rosnode list
/rosout
/turtlesim
aluno@aluno-VirtualBox:~$ rosnode info /turtlesim
--------------------------------------------------------------------------------
Node [/turtlesim]
Publications:
 * /rosout [rosgraph_msgs/Log]
 * /turtle1/color_sensor [turtlesim/Color]
 * /turtle1/pose [turtlesim/Pose]

Subscriptions:
 * /turtle1/cmd_vel [unknown type]

Services:
 * /clear
 * /kill
 * /reset
 * /spawn
 * /turtle1/set_pen
 * /turtle1/teleport_absolute
 * /turtle1/teleport_relative
 * /turtlesim/get_loggers
 * /turtlesim/set_logger_level


contacting node http://aluno-VirtualBox:36609/ ...
Pid: 10231
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (46529 - 127.0.0.1:41620) [18]
    * transport: TCPROS
```

O service ```/turtle1/teleport_absolute``` parece ser o que estamos procurando.

Na [roswiki](http://wiki.ros.org/turtlesim) procurando a descrição desse service e lendo a descrição contida na documentação confirmamos que
esse service é de fato o que queremos.

Como chamar o service em um script python?
As opções são: Achar uma explicação boa na Roswiki, achar um bom tutorial na internet/youtube ou adaptar o código 
de alguém.

No caso, por falta de tutorial mais simples, escolhi adaptar o script contido no seguinte [repositório](https://github.com/huchunxu/ros_21_tutorials/blob/master/learning_service/scripts/turtle_spawn.py). Os comandos originais
foram mantidos comentados seguidos dos comandos adaptados para o nosso caso do teleport.

Para descobrir como importar as bibliotecas para chamar o service pesquisei as informações da seguinte maneira.
```bash
aluno@aluno-VirtualBox:~$ rosservice list
/clear
/kill
/reset
/rosout/get_loggers
/rosout/set_logger_level
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/get_loggers
/turtlesim/set_logger_level
aluno@aluno-VirtualBox:~$ rosservice info /turtle1/teleport_absolute
Node: /turtlesim
URI: rosrpc://aluno-VirtualBox:46613
Type: turtlesim/TeleportAbsolute
Args: x y theta
```

E observando-se o atributo Type, bastou utilizar
```
from <tipo>.srv import <subtipo>
```
no caso
```
from turtlesim.srv import TeleportAbsolute
```
passando como argumentos
```
teleport_turtle = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
x = 8
y = 8
theta = 90 
teleport_turtle(x,y,theta)
```

```python
#!/usr/bin/env python

import sys
import rospy
#from turtlesim.srv import Spawn
from turtlesim.srv import TeleportAbsolute

#def turtle_spawn():
def turtle_gopose():
    
    #rospy.init_node('turtle_spawn')
    rospy.init_node('turtle_teleport')

    #rospy.wait_for_service('/spawn')
    rospy.wait_for_service('/turtle1/teleport_absolute')
    
    try:
        #add_turtle = rospy.ServiceProxy('/spawn', Spawn)
        teleport_turtle = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)

        #response = add_turtle(2.0, 2.0, 0.0, "turtle2")
        teleport_turtle(8,8,90)
        return 
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    
    #print "Spwan turtle successfully [name:%s]" %(turtle_spawn())
    turtle_gopose()
 
```

Para entender o que faz cada parte do programa veja [Código comentado](https://github.com/SkyRats/psi3442/blob/master/Curso2023/3a_Aula/Respostas/turtle_go_service.py).

## 5 Problema (1)
Escreva um script no arquivo ```turtle_go_topic.py``` criando uma rotina que quando chamada através do comando 

```
rosrun turtle_control turtle_go_topic.py
```

Seu programa deve perguntar no terminal a um usuário coordenadas desejadas (x,y) de modo que a tartagura trutle1 vá para essa posição usando a comunicação assíncrona via topics.

```diff
- Atenção!
- Para aproveitar o material da melhor forma possível
- resolva o exercício sem olhar a resposta!
```

## 6 Solução via comunicação Assíncrona
### 6.1 Programa em Python
[Resposta Assíncrona aqui](https://github.com/SkyRats/psi3442/blob/master/Curso2023/3a_Aula/Respostas/turtle_go_topic.py)
### 7.2 Execução do programa
Terminal 1
```
roscore
```
Terminal 2
```
rosrun turtlesim turtlesim_node
```
Terminal 3
```
rosrun turtle_control turtle_go_topic.py
```
## 8 Solução via comunicação Síncrona
### 8.1 Programa em Python
vide seção 4.2 dessa atividade

[Resposta Síncrona aqui](https://github.com/SkyRats/psi3442/blob/master/Curso2023/3a_Aula/Respostas/turtle_go_service.py)
### 8.2 Execução do programa
Terminal 1
```
roscore
```
Terminal 2
```
rosrun turtlesim turtlesim_node
```
Terminal 3
```
rosrun turtle_control turtle_go_service.py
```

## Importante :warning:

O ROS é uma ferramenta e por tanto a melhor forma de aprender a mexer com ela é praticando, a final de contas ferramentas servem para serem usadas.

O exercício 9 a seguir te ajudará a relembrar e fixar os conceitos da aula de hoje mas...

Se você quer se tornar um ninja da robótica :monocle_face: :dagger: , o exercício 10 é essencial para você pois ele introduz conceitos de posicionamento que são fundamentasis para uma tartaruga ninja politécnica que almeja ganhar o mundo na área de robótica com sistemas embarcados!

É hora de praticar!!!

## 9 Exercício de fixação do conteúdo 

[Contexto do exercício](https://pt.wikipedia.org/wiki/Tartarugas_Ninja)

Crie um pacote ROS chamado tartarugas_ninja onde uma tartaruga chamada Donatello :turtle: segue outra tartaruga chamada Leonardo :turtle:

Leonardo é o Líder do grupo. Ele se move de maneira determinada e sempre liderando o grupo. 
Seu comportamento por tanto se baseará em uma movimentação sincrona por meio do service teleport_absolute e a sua posição deve ser gerada por um gerador aleatório de números inteiros do python. As posições de Leonardo devem ser (x,y,0) onde x,y $\in$ [1,10]. A posição de Leonardo deve ser atualizada uma vez a cada 5s. O Leonardo será comandado pelo script leonardo.py

Donatello é outro membro do grupo. Sempre pronto para a ação ele segue Leonardo sempre indo para a posição (x-1,y-1) onde (x,y) é a posição de Leonardo. Assim, seu irmão tem espaço para batalhar contra os inimigos e Donatello sempre está lá para protege-lo. Donatello deve se mover observando assincronamente a movimentação de Leonardo por meio do topic de posição que contém as posição atual de Leonador. A movimentação de Donatello deve se dar publicando comandos de velocidade no topico de velocidade associado a movimentação de Donatello. O Donatello será comandado pelo script donatello.py 

## 10 Exercício Ninja da robótica :sunglasses: :sunglasses:

Altere o exercío 9 com as seguintes modificações em novos scripts conforme indicado a seguir.

Leonardo :turtle: :
calcule e aplique o angulo theta de Leonardo para que ele esteja sempre posicionado para paralelamente a sua trajetória, isto é, para que saindo de (x0,y0) ele chegue em (x1,y1) sempre olhando para frente, isto é, no sentido oposto do vetor (x1,y1)-(x0,y0). O Leonardo será comandado pelo script leonardo_lider_nato.py

Donatello :turtle: :
introduza uma função extra de rotação no Donatello para que ele quando ele chegue na coordenada (x-1,y-1) ele realize uma rotação em torno do próprio centro de massa a mais de modo que Donatello curba a retaguarda de Leonardo, isto é, de modo que Donatello fique de costas para Leonardo. O Donatello será comandado pelo script donatello_fiel_escudeiro.py

![tartarugasninja](https://github.com/SkyRats/psi3442/blob/master/Curso2023/3a_Aula/images/gifs-de-tartarugas-ninjas-36.gif)

