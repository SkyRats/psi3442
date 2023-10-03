# Simulação Software in the Looop de um drone IRIS com Firware PX4, MAVROS/MAVLINK e Gazebo

A documentação a seguir Explica em detalhes como criar e customizar simulações SITL (Software in the loop) usando o PX4, MAVROS/MAVLINK e Gazebo.

Para os tutoriais a seguir, alguns conhecimentos são necessários e podem ser encontrados nos seguintes links. Consulte-os quando necessário. Recomendamos tentar entender primeiro os tutoriais da skyrats e recorer a estes links quando necessário.

* SDF: Simulation Description File

[Documentação SDF](http://sdformat.org/tutorials?cat=specification&)

Aqui você poderá encontrar os significados de cada <tag> usada nesta linguagem de descrição de arquivos de simulação e entender o significado de argumentos e parâmetros configurados nos arquivos de simulação.

* Gazebo

[Tutorias gazebo](https://classic.gazebosim.org/tutorials)

Aqui você pode conferir os tutoriais oficiais do gazebo para te auxiliar no entendimento de algo que não tenha ficado claro nas explicações das seções deste tutorial.

* PX4
  
[Tutorial do firmware px4 de como rodar uma simulação autonoma](https://docs.px4.io/main/en/ros/mavros_offboard_python.html)

Aqui você pode conferir a documentação oficial da PX4 sobre como rodar uma simulação autonoma com dorne IRIS e o Gazebo.

## Parte 1 Entendendo o Gazebo

![rqt_graph](images/gazebosim.png)

No link a seguir você terá a oportunidade de aprender sobre o Gazebo. Observação: Arquivos mencionados no tutorial encontram-se nesse mesmo link do github.

[Siga para o tutorial Skyrats sobre Gazebo](https://github.com/SkyRats/psi3442/tree/master/5a_Aula)

## Parte 2 Utilizando Sensores (LiDAR e Câmera)

No link a seguir você terá a oportunidade de aprender sobre o Sensores simulados no Gazebo. Observação: Arquivos mencionados no tutorial encontram-se nesse mesmo link do github.

[Siga para o tutorial Skyrats sobre Sensores](https://github.com/SkyRats/psi3442/tree/master/6a_Aula/scripts)

## Parte 3 Customizando tudo

Nesta terceira parte do tutorial você terá a oportunidade de aprender a criar o pacote iris_sim que utilizará modelos customizados por você salvos no diretório ~/.gazebo/models feitos no Blender para criar as simulações mais incríveis com drones!

Na aula de hoje veremos como criar um mundo gramado, com um lindo céu com nuvens brancas, alguns objetos de cena em um dia ensolarado e com dois balões incríveis.

Observação importante: O tutorial a seguir se propõe a explicar como criar os arquivos e organizá-los de forma correta para que a simulação funcione. Porém, os códigos dos objetos, modelos e mundos cirados não serão apresentados no texto. Fica a cargo do leitor conferir o diretório .gazebo/models e o rospackge iris_sim (ambos fornecidos nesse turorial) e procurar os arquivos mencionados neste tutorail para entender na prática o que é explicado neste texto.

### 3.1 Blender

![rqt_graph](images/blender.png)

O [Blender](https://www.blender.org/) é uma ferramenta de modelagem e desenho 3D fácil de usar e que lhe permitirá criar obejtos e cenários para sua simulação no gazebo.

Caso queria aprender mais sobre como desenhar no Blender recomendo o seguinte [tutorial](https://www.youtube.com/watch?v=UAami_DhnTA&list=PLC7nmYI-cbT1gLvOzU-pcIZKbPezbRSyz). Se preferir aprender a partir do zero mas com um exemplo mais completo confira este [vídeo](https://www.youtube.com/watch?v=YjyObVcdHZY) do mesmo canal.

Ok, sabendo desenhar o que você deseja no Blender, é hora de importar seu modelo para o Gazebo.

Para esse tutorial eu criei um objeto chamado crazy_object.blend (arquivo do blender) mostrado a seguir.

![rqt_graph](images/crazy_object.png). Agora, para importar para o gazebo devemos gerar um objeto crazy_object.dae com essa nova extensão de arquivos. 

Para isto, basta exportar o projeto crazy_object no Blender escolhendo a opção .dae e pronto. Com o arquivo crazy_object.dae podemos criar um modelo no diretório ~/.gazebo/models/ chamado crazy_object. Este modelo poderá ser chamado no arquivo mundo_customizado.world

## 3.2 IRIS SIM! Um universo onde sou amigo do Rei!

## 3.3 Brilha sol! Ilumine meu mundo!

## 3.4 Um Belo Gramado

## 3.5 O Céu Perfeito

## 3.6 Adicionando um objeto inusitado

Coloquei um objeto impróprio na minha simulação, veja no que deu [youtube]()

## 3.7 o drone que eu sempre quis!

## 3.8 Vendo o mundo de cima! A melhor câmera de todas!

## 3.9 Um poder de semi-deus. Roslaunch, big-bang! Faça-se a luz! É hora de iniciar minha criação!
