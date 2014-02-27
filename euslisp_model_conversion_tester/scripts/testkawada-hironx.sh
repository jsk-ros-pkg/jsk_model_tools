#!/bin/bash

. $HOME/.bashrc

ROBOT=hironx
rosrun euslisp_model_conversion_tester testROBOT.sh -c `rospack find collada_robots`/data/robots/kawada-hironx.dae -e `rospack find hrpsys_ros_bridge_tutorials`/models/kawada-hironx.l -r ${ROBOT}

