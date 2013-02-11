#!/bin/bash

if [ $# -ge 1 ]; then
    ROBOTFILE=$1;
else
    ROBOTFILE=$(rospack find hrpsys_ros_bridge)/models/HRP4C.dae;
fi

script/pub_joint_trajectory_action.py --joints=$(sed -n -e 's@.*<joint name="\([^ ]*\)".*@\1,@p' $ROBOTFILE | sed ':lbl1;N;s/\n//;b lbl1;' | sed -e 's@,$@@') --action=/joint_traj_controller/joint_trajectory_action
