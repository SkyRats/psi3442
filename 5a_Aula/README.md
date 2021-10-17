# Aula 5 - Gazebo e Sensores

Nesta atividade, vamos explorar o ambiente de simulação utilizado, o Gazebo, e a utilização de sensores em sistemas embarcados.

## 1. Gazebo

O [Gazebo](http://gazebosim.org/) é um simulador Open Source amplamente utilizado em robótica. 

### 1.1. Models

**Exemplo: Asphalt plane**

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="asphalt_plane">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>200 200 .1</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry>
          <box>
            <size>200 200 .1</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>model://asphalt_plane/materials/scripts</uri>
            <uri>model://asphalt_plane/materials/textures</uri>
            <name>vrc/asphalt</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

### Exemplo: empty.world

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://asphalt_plane</uri>
    </include>
    <physics name='default_physics' default='0' type='ode'>
      <gravity>0 0 -9.8066</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
          <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
      <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>
    </physics>
  </world>
</sdf>
```

## 2. Sensores

### Exemplo: Drone com câmera

```xml
<?xml version='1.0'?>
<sdf version='1.5'>
  <model name='iris_fpv_cam'>

    <include>
      <uri>model://iris</uri>
    </include>

    <include>
      <uri>model://fpv_cam</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
    <joint name="fpv_cam_joint" type="fixed">
      <child>fpv_cam::link</child>
      <parent>iris::base_link</parent>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <upper>0</upper>
          <lower>0</lower>
        </limit>
      </axis>
    </joint>

  </model>
</sdf>
```