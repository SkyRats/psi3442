# Simulação Software in the Looop de um drone IRIS com Firware PX4, MAVROS/MAVLINK e Gazebo

A documentação a seguir Explica em detalhes como criar e customizar simulações SITL (Software in the loop) usando o PX4, MAVROS/MAVLINK e Gazebo.

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

### 3.1 Blender

![rqt_graph](images/blender.png)

O [Blender](https://www.blender.org/) é uma ferramenta de modelagem e desenho 3D fácil de usar e que lhe permitirá criar obejtos e cenários para sua simulação no gazebo.

Caso queria aprender mais sobre como desenhar no Blender recomendo o seguinte [tutorial](https://www.youtube.com/watch?v=UAami_DhnTA&list=PLC7nmYI-cbT1gLvOzU-pcIZKbPezbRSyz). Se preferir aprender a partir do zero mas com um exemplo mais completo confira este [vídeo](https://www.youtube.com/watch?v=YjyObVcdHZY) do mesmo canal.

Ok, sabendo desenhar o que você deseja no Blender, é hora de importar seu modelo para o Gazebo.

Para esse tutorial eu criei 
