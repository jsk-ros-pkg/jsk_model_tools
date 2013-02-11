#!/bin/bash

PS1=gazebo
source ${HOME}/.bashrc

if [ $# -ge 2 ]; then
    dname=`rospack find $1`/`dirname $2`
    tmp_filename=$(basename "$2")
    rname=`dirname $2`
    bname=`basename $2`
    filetype="${tmp_filename##*.}"
    fname="${tmp_filename%.*}"
    ##fname=$(echo $bname | sed -n -e "s/\(.*\)\.${filetype}/\1/p")

    filepos=${dname}/${fname}.${filetype}
    if [ -n ${fname} -a -e ${filepos} ]; then
        case ${filetype} in
            "urdf") \
                if [ "`grep -c 'xmlns:xacro' ${filepos}`" -eq "0" ]; then
                    sed -i -e "s@\(<robot name=.*\)@\1 xmlns:xacro=\"http://www.ros.org/wiki/xacro\">@" ${filepos}
                fi
                sed -e "s@</robot>@<xacro:include filename=\"\$(find $1)/${rname}/${fname}_gazebo.xacro\" />\n</robot>@" ${filepos} > ${dname}/${fname}.gazebo.${filetype}.xacro ;;
            "dae") \
                sed -e "s@<COLLADA xmlns=\"http://www.collada.org/2008/03/COLLADASchema\" version=\"1.5.0\">@<COLLADA xmlns=\"http://www.collada.org/2008/03/COLLADASchema\" version=\"1.5.0\"\nxmlns:xacro=\"http://www.ros.org/wiki/xacro\">@" ${filepos} | \
        sed -e "s@</COLLADA>@<xacro:include filename=\"\$(find $1)/${rname}/${fname}_gazebo.xacro\" />\n</COLLADA>@" > ${dname}/${fname}.gazebo.dae.xacro ;;
            *) echo "unsupported file type ${filetype}"; exit 1 ;;
        esac
        (roscd $1; rosrun collada_tools collada_gazebo_gen $1 ${rname} ${fname} ${filetype})
        rosrun xacro xacro.py ${dname}/${fname}.gazebo.urdf.xacro > ${dname}/${fname}.gazebo.${filetype}
        cat > ${dname}/${fname}_gazebo.launch <<EOF
<launch>
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="throttled" default="false"/>
  <arg name="z_offset" default="0.0" />

  <!-- start world -->
  <include file="\$(find gazebo_worlds)/launch/empty_world.launch" >
    <arg name="paused" value="\$(arg paused)"/>
    <arg name="use_sim_time" value="\$(arg use_sim_time)"/>
    <arg name="gui" value="\$(arg gui)"/>
    <arg name="throttled" value="\$(arg throttled)"/>
  </include>

  <!-- push robot_description to factory and spawn robot in gazebo -->
  <param name="robot_description"
         textfile="\$(find $1)/${rname}/${fname}.gazebo.${filetype}" />
  <node name="${fname}_robot_spawn" pkg="gazebo" type="spawn_model"
        args="-z \$(arg z_offset) -urdf -param robot_description -model ${fname}"
        respawn="false" output="screen" />

  <!-- state publisher -->
  <node pkg="robot_state_publisher" type="state_publisher"
        name="${fname}_robot_stat_publisher" />

  <!-- for controller -->
  <rosparam file="\$(find $1)/${rname}/${fname}_controllers.yaml" command="load"/>
  <node name="spawn_trj_controller" pkg="pr2_controller_manager" type="spawner"
        args="fullbody_controller" respawn="false" output="screen" />

 <!--node pkg="rviz" type="rviz" name="rviz" args="-d \$(find $1)/${rname}/rviz_${fname}.vcg"/-->
</launch>
EOF
    else
        # error message
        echo "${filepos} is not .${filetype} file or ${filepos} does not exist"
        exit 1
    fi
    if [ $# -ge 3 ]; then
        echo "output_dir $3"
    fi
else
    # error message
    echo "usage: $(basename $0) packagename path_to/file.${filetype}"
    exit 1
fi
exit 0
