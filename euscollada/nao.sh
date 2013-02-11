#!/bin/bash

#rosrun collada_urdf_jsk_patch urdf_to_collada `rospack find nao_description`/urdf/nao_robot.xml nao.dae
#rosrun xacro xacro.py  -o nao_robot_v3_structure.urdf `rospack find nao_description`/urdf/nao_robot_v3_structure.urdf.xacro
rosrun xacro xacro.py  -o nao_robot_v3.urdf `rospack find nao_description`/urdf/nao_robot_v3.urdf.xacro
sed -i s/torso/torso_link/ nao_robot_v3.urdf
rosrun collada_urdf urdf_to_collada nao_robot_v3.urdf nao.dae
#rosrun collada_urdf_jsk_patch urdf_to_collada nao_robot_v3.urdf nao.dae
#rosrun collada_urdf urdf_to_collada `rospack find nao_description`/urdf/nao_robot.xml nao.dae
if [ "$?" != 0 ] ;  then exit ; fi

rosrun euscollada collada2eus nao.dae nao.yaml nao.l
if [ "$?" != 0 ] ;  then exit ; fi

rosrun euslisp irteusgl -e "(progn (load \"nao.l\")(nao)(make-irtviewer)(objects (list *nao*)))"
