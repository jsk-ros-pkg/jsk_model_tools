#!/bin/bash

. $HOME/.bashrc

while getopts "hv:c:e:r:" flag; do
  case $flag in
    \?) OPT_ERROR=1; break;;
    v) VRML_INPUTFILE="$OPTARG";;
    c) COLLADA_INPUTFILE="$OPTARG";;
    e) EUSLISP_INPUTFILE="$OPTARG";;
    r) ROBOT="$OPTARG";;
    h) HELP=true;;
  esac
done
function print-help {
    echo "usage : $0 [options] old-user new-user";
    echo "[options]";
    echo "  -h   :  print help";
    echo "  -v [VRML_INPUTFILE] : specify VRML_INPUTFILE";
    echo "  -r [ROBOT] : specify ROBOT";
    echo "  -c [COLLADA_INPUTFILE] : specify COLLADA_INPUTFILE";
    echo "  -e [EUSLISP_INPUTFILE] : specify EUSLISp_INPUTFILE";
}
if [ $HELP ];then print-help; exit; fi;

robot=`echo $ROBOT | tr "[A-Z]" "[a-z]"`
if [ "$EUSLISP_INPUTFILE" == "" ];then EUSLISP_INPUTFILE=`rospack find hrpsys_ros_bridge_tutorials`/models/${robot}.l;fi
if [ "$COLLADA_INPUTFILE" == "" ];then COLLADA_INPUTFILE=`rospack find hrpsys_ros_bridge_tutorials`/models/${ROBOT}.dae;fi
EUSLISP_OUTPUTFILE=/tmp/${ROBOT}_euslisp_robot_model.yaml
OPENHRP3_VRML_OUTPUTFILE=/tmp/${ROBOT}_openhrp3_vrml_robot_model.yaml
OPENHRP3_COLLADA_OUTPUTFILE=/tmp/${ROBOT}_openhrp3_collada_robot_model.yaml

if [ "$VRML_INPUTFILE" != "" ];
then
    rosrun hrpsys_ros_bridge rtmtest -t euslisp_model_conversion_tester gen_openhrp3_robot_model_yaml.launch INPUT:=$VRML_INPUTFILE OUTPUT:=$OPENHRP3_VRML_OUTPUTFILE
fi
rosrun hrpsys_ros_bridge rtmtest -t euslisp_model_conversion_tester gen_openhrp3_robot_model_yaml.launch INPUT:=$COLLADA_INPUTFILE OUTPUT:=$OPENHRP3_COLLADA_OUTPUTFILE
roseus `rospack find euslisp_model_conversion_tester`/euslisp/write-euslisp-robot-model.l "(load \"${EUSLISP_INPUTFILE}\")" "(${robot})" "(write-robot-model-yaml \"${EUSLISP_OUTPUTFILE}\" *${robot}*)" "(exit 0)"

# test
if [ "$EUSLISP_INPUTFILE" != "" ];
then
    if [ "$VRML_INPUTFILE" != "" ];
    then
        rosrun euslisp_model_conversion_tester compare_robot_model_yaml.py $OPENHRP3_VRML_OUTPUTFILE $EUSLISP_OUTPUTFILE
    else
        rosrun euslisp_model_conversion_tester compare_robot_model_yaml.py $OPENHRP3_COLLADA_OUTPUTFILE $EUSLISP_OUTPUTFILE
    fi
fi
if [ "$COLLADA_INPUTFILE" != "" -a "$VRML_INPUTFILE" != "" ];
then
    rosrun euslisp_model_conversion_tester compare_robot_model_yaml.py $OPENHRP3_COLLADA_OUTPUTFILE $OPENHRP3_VRML_OUTPUTFILE
fi
