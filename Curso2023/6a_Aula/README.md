# Customizando uma simulação de Drone usando firmware PX4, ROS Melodic e Gazebo 9

Nesssa atividade iremos:
* Criar um novo modelo de drone no Gazebo
* Adicionar uma câmera
* Testar a captura de Imagem


## 1 Criando um novo modelo de drone no Gazebo

Para criar um modelo customizado de Drone para rodar simulações usando o Firmware PX4, ROS Melodic e Gazebo 9 vamos utilizar como base o empty.world do Gazebo e o Drone IRIS pardão. 

Não podemos mexer no modelo já existente no firmware, por isso iremos fazer uma cópia do Drone IRIS e depois alterá-lo.

Fazer isso não é muito trivial pois envolve alterar varios arquivos. Por isso, siga todos os passos do tutorial a seguir.

Vamos criar um novo [airframe](https://docs.px4.io/main/en/dev_airframes/adding_a_new_frame.html).Realizea seguinte sequência de passos:

Todos os caminhos de diretório mencionados a seguir começam na pasta principal do seu Firmware PX4. 

Por exemplo: Firware/Tools/sitl_gazebo ...

se você nomeou seu diretório do firware PX4 como "Firmware".

Observação: No tutorial a seguir é criado um novo airframe chamado "my_vehicle". Mude esse nome para um que faça mais sentido para você. E prefira nomes do tipo "palavra_outrapalavra" com apenas um underscore.

1) Create a folder under Tools/sitl_gazebo/models for your model, let’s call it my_vehicle
   
2) Create the following files under Tools/sitl_gazebo/models/my_vehicle: model.config and my_vehicle.sdf (these can be based off the iris or solo models in Tools/sitl_gazebo/models)

No passo 2, se você for copiar o Drone IRIS padrão copie os seguintes arquivos:

iris.sdf (que contém o modelo do gazebo do Drone IRIS)

E 

model.config (que contém uma descrição de informações básicas do modelo)

Lembre-se de reonemar os arquivos copiados para my_vehicle ou para o nome que você escolheu.

3) Create a world file in Tools/sitl_gazebo/worlds called my_vehicle.world (Again, this can be based off the iris or solo world files)

No passo 3 uma boa ideia é fazer uma cópia do empty.world para começar e para que sua simulação não fique muito pesada com elementos desnecessários contidos nos outros mundos. Lembre-se de reonemar os arquivos copiados para my_vehicle ou para o nome que você escolheu.
   
4) Create an airframe file under ROMFS/px4fmu_common/init.d-posix (Yet again, this can be based off the iris or solo airframe files), give it a number (for example 4229) and name it 4229_my_vehicle

No passo 4, sugiro copiar o arquivo do iris (talvez no seu computador o nome do arquivo seja 10016_iris). Use como base o modelo mais simples do Drone IRIS).Lembre-se de reonemar os arquivos copiados para my_vehicle ou para o nome que você escolheu.
  
5) Add the airframe name (my_vehicle) to the file platforms/posix/cmake/sitl_target.cmake in the command set(models …

No passo 5,altere a função set() do arquivo indicado. Você deve alterar o arquivo na seção similar ao exemplo a seguir.Lembre-se de reonemar os arquivos copiados para my_vehicle ou para o nome que você escolheu

``` cmake
set(models none shell
	if750a iris iris_citiusp iris_stereo_camera iris_dual_gps iris_opt_flow iris_opt_flow_mockup iris_vision iris_rplidar iris_irlock iris_obs_avoid iris_rtps px4vision solo typhoon_h480
	plane plane_cam plane_catapult plane_lidar techpod
	standard_vtol tailsitter tiltrotor
	rover r1_rover boat cloudship
	uuv_hippocampus)
```
   
6) Add the airframe name "4229_my_vehicle" in the CMakeLists.txt located in ROMFS/px4fmu_common/init.d-posix/airframes   

No passo 6, você deve ver algo como 

```
px4_add_romfs_files(
	10016_iris
	10020_if750a
	10030_px4vision
	10070_iris_citiusp
```

Now you are free to tweak any of the model values in the SDF file and any of the controller gains in the airframe file.

You can launch SITL with your model with “make px4_sitl gazebo_my_vehicle” 



