jsk_model_tools [![Build Status](https://travis-ci.org/jsk-ros-pkg/jsk_model_tools.png?branch=master)](https://travis-ci.org/jsk-ros-pkg/jsk_model_tools)
===============

| Package                 | Indigo (Saucy)                                                                                                                                                                                         | Indigo (Trusty)                                                                                                                                                                                          | Jade (Trusty)                                                                                                                                                                                            | Jade (Vivid)                                                                                                                                                                                           | Kinetic (Wily)                                                                                                                                                                                       | Kinetic (Xenial)                                                                                                                                                                                           |
|:------------------------|:-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|:-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|:-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|:-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| jsk_model_tools (armhf) | [![Build Status](http://build.ros.org/job/Ibin_arm_uShf__jsk_model_tools__ubuntu_saucy_armhf__binary/badge/icon)](http://build.ros.org/job/Ibin_arm_uShf__jsk_model_tools__ubuntu_saucy_armhf__binary) | [![Build Status](http://build.ros.org/job/Ibin_arm_uThf__jsk_model_tools__ubuntu_trusty_armhf__binary/badge/icon)](http://build.ros.org/job/Ibin_arm_uThf__jsk_model_tools__ubuntu_trusty_armhf__binary) | [![Build Status](http://build.ros.org/job/Jbin_arm_uThf__jsk_model_tools__ubuntu_trusty_armhf__binary/badge/icon)](http://build.ros.org/job/Jbin_arm_uThf__jsk_model_tools__ubuntu_trusty_armhf__binary) | [![Build Status](http://build.ros.org/job/Jbin_arm_uVhf__jsk_model_tools__ubuntu_vivid_armhf__binary/badge/icon)](http://build.ros.org/job/Jbin_arm_uVhf__jsk_model_tools__ubuntu_vivid_armhf__binary) | [![Build Status](http://build.ros.org/job/Kbin_arm_uWhf__jsk_model_tools__ubuntu_wily_armhf__binary/badge/icon)](http://build.ros.org/job/Kbin_arm_uWhf__jsk_model_tools__ubuntu_wily_armhf__binary) | [![Build Status](http://build.ros.org/job/Kbin_uxhf_uXhf__jsk_model_tools__ubuntu_xenial_armhf__binary/badge/icon)](http://build.ros.org/job/Kbin_uxhf_uXhf__jsk_model_tools__ubuntu_xenial_armhf__binary) |
| jsk_model_tools (i386)  | [![Build Status](http://build.ros.org/job/Ibin_uS32__jsk_model_tools__ubuntu_saucy_i386__binary/badge/icon)](http://build.ros.org/job/Ibin_uS32__jsk_model_tools__ubuntu_saucy_i386__binary)           | [![Build Status](http://build.ros.org/job/Ibin_uT32__jsk_model_tools__ubuntu_trusty_i386__binary/badge/icon)](http://build.ros.org/job/Ibin_uT32__jsk_model_tools__ubuntu_trusty_i386__binary)           | [![Build Status](http://build.ros.org/job/Jbin_uT32__jsk_model_tools__ubuntu_trusty_i386__binary/badge/icon)](http://build.ros.org/job/Jbin_uT32__jsk_model_tools__ubuntu_trusty_i386__binary)           | [![Build Status](http://build.ros.org/job/Jbin_uV32__jsk_model_tools__ubuntu_vivid_i386__binary/badge/icon)](http://build.ros.org/job/Jbin_uV32__jsk_model_tools__ubuntu_vivid_i386__binary)           | [![Build Status](http://build.ros.org/job/Kbin_uW32__jsk_model_tools__ubuntu_wily_i386__binary/badge/icon)](http://build.ros.org/job/Kbin_uW32__jsk_model_tools__ubuntu_wily_i386__binary)           | [![Build Status](http://build.ros.org/job/Kbin_uX32__jsk_model_tools__ubuntu_xenial_i386__binary/badge/icon)](http://build.ros.org/job/Kbin_uX32__jsk_model_tools__ubuntu_xenial_i386__binary)             |
| jsk_model_tools (amd64) | [![Build Status](http://build.ros.org/job/Ibin_uS64__jsk_model_tools__ubuntu_saucy_amd64__binary/badge/icon)](http://build.ros.org/job/Ibin_uS64__jsk_model_tools__ubuntu_saucy_amd64__binary)         | [![Build Status](http://build.ros.org/job/Ibin_uT64__jsk_model_tools__ubuntu_trusty_amd64__binary/badge/icon)](http://build.ros.org/job/Ibin_uT64__jsk_model_tools__ubuntu_trusty_amd64__binary)         | [![Build Status](http://build.ros.org/job/Jbin_uT64__jsk_model_tools__ubuntu_trusty_amd64__binary/badge/icon)](http://build.ros.org/job/Jbin_uT64__jsk_model_tools__ubuntu_trusty_amd64__binary)         | [![Build Status](http://build.ros.org/job/Jbin_uV64__jsk_model_tools__ubuntu_vivid_amd64__binary/badge/icon)](http://build.ros.org/job/Jbin_uV64__jsk_model_tools__ubuntu_vivid_amd64__binary)         | [![Build Status](http://build.ros.org/job/Kbin_uW64__jsk_model_tools__ubuntu_wily_amd64__binary/badge/icon)](http://build.ros.org/job/Kbin_uW64__jsk_model_tools__ubuntu_wily_amd64__binary)         | [![Build Status](http://build.ros.org/job/Kbin_uX64__jsk_model_tools__ubuntu_xenial_amd64__binary/badge/icon)](http://build.ros.org/job/Kbin_uX64__jsk_model_tools__ubuntu_xenial_amd64__binary)           |

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
rosrun euscollada collada2eus my_model.dae my_model.l
```
=> my_model.l is generated.

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
