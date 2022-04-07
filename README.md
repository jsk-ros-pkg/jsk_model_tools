jsk_model_tools [![Build Status](https://app.travis-ci.com/jsk-ros-pkg/jsk_model_tools.svg?branch=master)](https://app.travis-ci.com/jsk-ros-pkg/jsk_model_tools)
===============

# Model Convert

## Convert from CAD manually
(cf. PR of function to automatically convert: https://github.com/euslisp/jskeus/pull/248)

- Convert1. export stl mesh with SolidEdge

=> my_model.stl is generated.

- Convert2. generate urdf
generate urdf with following contents.
（assume that my_model.stl is located at /home/leus/my_model.stl）
```
<robot name="my_model_name">
  <link name="root">
    <inertial>
     <origin xyz="0 0 0.5" rpy="0 0 0"/>
     <mass value="1"/>
     <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
    </inertial>
    <visual>
     <origin xyz="0 0 0" rpy="0 0 0" />
     <geometry>
       <mesh filename="file:///home/leus/my_model.stl" />
     </geometry>
    </visual>
    <collision>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <geometry>
       <mesh filename="file:///home/leus/my_model.stl" />
     </geometry>
    </collision>
  </link>
</robot>
```
=> save to my_model.urdf

- Convert3. convert to collada
```
rosrun collada_urdf_jsk_patch urdf_to_collada my_model.urdf my_model.dae
```
=> my_model.dae is generated.

- Convert4. convert to eus
```
rosrun euscollada collada2eus my_model.dae my_model.l # fixed model version
rosrun euscollada collada2eus my_model.dae my_model.yaml my_model.l
```
=> my_model.l is generated.\
my_model.yaml is used to set joints and endcoords settings.
Please refer to [example file of PR2](euscollada/pr2.yaml).\
Without the yaml file, the model can not move or get/send `:angle-vector`.

- Visualize1. visualize urdf with Rviz
```
roslaunch urdf_tutorial display.launch model:=/home/leus/my_model.urdf
```

- Visualize2. spawn urdf to gazebo
```
roslaunch gazebo_ros empty_world.launch
rosrun gazebo_ros spawn_model -file my_model.urdf -urdf -model my_model
```

- Visualize3. visualize eus with irtviewer
```
roseus my_model.l
(objects (list (my_model)))
```

- Visualize4. visualize stl with irtviewer
```
(load "package://eus_assimp/euslisp/eus-assimp.l" )
(setq glv (load-mesh-file "my_model.stl" :scale 1000.0))
(objects (list glv))
```
