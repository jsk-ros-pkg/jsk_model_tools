#!/bin/bash

PS1=gazebo
source ${HOME}/.bashrc

if [ $# -ge 2 ]; then
    dname=`rospack find $1`/`dirname $2`
    bname=`basename $2`
    rname=`dirname $2`
    fname=$(echo $bname | sed -n -e 's/\(.*\)\.dae/\1/p')
    filepos=${dname}/${fname}.dae
    if [ -n ${fname} -a -e $filepos ]; then
        sed -e "s@<COLLADA xmlns=\"http://www.collada.org/2008/03/COLLADASchema\" version=\"1.5.0\">@<COLLADA xmlns=\"http://www.collada.org/2008/03/COLLADASchema\" version=\"1.5.0\"\nxmlns:xacro=\"http://www.ros.org/wiki/xacro\">@" ${filepos} | \
        sed -e "s@</COLLADA>@<xacro:include filename=\"\$(find $1)/${rname}/${fname}_gazebo.xacro\" />\n</COLLADA>@" > ${dname}/${fname}.gazebo.dae.xacro && \
        (roscd $1; rosrun collada_gazebo_tools collada_gazebo_gen $1 ${rname} ${fname} dae) && \
        rosrun xacro xacro.py ${dname}/${fname}.gazebo.dae.xacro > ${dname}/${fname}.gazebo.dae
        cat > ${dname}/${fname}_gazebo.launch <<EOF
<launch>
  <arg name="gui" default="true"/>
  <arg name="throttled" default="false"/>
  <arg name="paused" default="true"/>

  <!-- start world -->
  <param name="/use_sim_time" value="true" />
  <include file="\$(find gazebo_worlds)/launch/empty_world_paused.launch">
    <arg name="gui" value="\$(arg gui)" />
    <arg name="throttled" value="\$(arg throttled)" />
    <arg name="paused" value="\$(arg paused)" />
  </include>

  <!-- push robot_description to factory and spawn robot in gazebo -->
  <param name="robot_description" textfile="\$(find $1)/${rname}/${fname}.gazebo.dae" />
  <node name="${fname}_robot_spawn" pkg="gazebo" type="spawn_model" args="-z 0.915 -urdf -param robot_description -model ${fname}" respawn="false" output="screen" />

  <!-- state publisher -->
  <node pkg="robot_state_publisher" type="state_publisher" name="rob_st_pub" />

  <!-- for controller -->
  <rosparam file="\$(find $1)/${rname}/${fname}_controllers.yaml" command="load"/>
  <node name="spawn_trj_controller" pkg="pr2_controller_manager" type="spawner" args="arm_controller" respawn="false" output="screen" />

 <!--node pkg="rviz" type="rviz" name="rviz" args="-d \$(find $1)/${rname}/rviz_${fname}.vcg"/-->
</launch>
EOF
    else
        # error message
        echo "${filepos} is not .dae file or ${filepos} does not exist"
        exit 1
    fi
    if [ $# -ge 3 ]; then
        echo "output_dir $3"
    fi
else
    # error message
    echo "usage: $(basename $0) packagename path_to/collada.dae"
    exit 1
fi
exit 0
