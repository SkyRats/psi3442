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

O iris_sim é um rospackge criado com rospy (python) e roscpp (C++, opicional caso queria programar seus scripts já em C++ pensando em aplicá-los de maneira otimizada em sistemas embarcados). O objetivo desse pacote é fornecer um arquivo chamado simulation.launch capaz prover um ambiente de simulação com um mundo customizado e drone customizado integrado ao firmware PX4.

Após criar o pacote adequadamente com os comandos ensinados no tutorial de ROS, incluimos as seguintes pastas adicionais para criar noss maravilhos pacote de simulação. Se você quiser aprender mais, você pode criar o pacote do zero e ir montando ele copiando e alterando os arquivos do pacote original. Se você tem pressa, pode baixar o pacote direto e apenas alterar aquilo que faz sentido para você.

Dentro do packge iris_sim você verá, dentre outros diretórios os seguintes:

* launch : guarda o arquivo simulation.launch responsável por chamar o firmware PX4, iniciar o ROS e Mavros. E iniciar o drone IRIS customizado dentro de um mundo também customizado. Para mais detalhes entre no arquivo simulation.launch

* models: contém o arquivo iris_custom. Este arquivo é um modelo .sdf customizável de um drone IRIS padrão que é instanciado e customizado por meio de modelos (sensores/atuadores) salvos no diretório ~/.gazebo/models

* scripts: contém os algoritimos de automação do drone. Você pode salvar seus scripts de automação em outro rospackge se preferir. Se por ventura achar mais conveniente salvar tudo junto, basta adicionar seu .py na pasta scripts.

* worlds: guarda o arquivo custom_world.world que descreve um mundo personalizável que uni modelos do diretório ~/.gazebo/models

## 3.3 Brilha sol! Ilumine meu mundo!

Aqui vamos entender o modelo sun_customized que está implementado no diretório ~/.gazebo/models. O modelo em si encontra-se no arquivo  ~/.gazebo/models/sun_customized/model.sdf

Define-se aqui uma fonte de luz perpendicular ao solo como sendo o sol do mundo outdoor do iris_sim. Para entender melhor as <tags> de iluminação leia [Parâmetros de iluminação em SDF](http://sdformat.org/tutorials?tut=spec_materials&cat=specification&)

Com isso, incluindo o modelo sun_customized no mundo custom_world.world o mundo passa a ser iluminado pelo sol com os parâmetros que você escolheu.

## 3.4 Um Belo Gramado

Aqui vamos entender o modelo grass_plane que está implementado no diretório ~/.gazebo/models. O modelo em si encontra-se no arquivo  ~/.gazebo/models/grass_plane/model.sdf

Define-se aqui o material e especificações do solo do mundo outdoor do iris_sim. Este modelo é muito similar ao asphalt_plane explicado na parte 1 deste tutorial. Basicamente o modelo conta com um arquivo model.sdf onde a descrição principal do modelo de piso gramado é feita. A pasta "materials" contém duas outas: "sccipts" e "textures".
A pasta "scripts" tem um arquivo chamado grass.material . Este arquivo define o material grama, a sua textura salva no formato .png e como a luz afeta este material. A pasta "textures" contém uma foto chamada grass.png conténdo uma visão aérea de um chão gramado quadrado.

Com isso, incluindo o modelo grass_plane no mundo custom_world.world o piso do mundo passa a ser um belo gramado verde.

## 3.5 O Céu Perfeito

``` xml
<scene>
  <sky>
    <clouds>
      <speed>12</speed>
    </clouds>
  </sky>
  <ambient>0.95 0.95 0.95 1</ambient>
  <background>0.3 0.3 0.3 1</background>
  <shadows>true</shadows>
</scene>
```

## 3.6 Adicionando um objeto inusitado

## 3.7 o drone que eu sempre quis!

## 3.8 Vendo o mundo de cima! A melhor câmera de todas!

## 3.9 Um poder de semi-deus. Roslaunch, big-bang! Faça-se a luz! É hora de iniciar minha criação!
