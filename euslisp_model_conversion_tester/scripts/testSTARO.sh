#!/bin/bash

. $HOME/.bashrc

ROBOT=STARO
rosrun euslisp_model_conversion_tester testROBOT.sh -v $CVSDIR/euslib/rbrain/staro/STAROmain.wrl -c `rospack find jsk_hrpsys_ros_bridge`/models/STARO.dae -e `rospack find jsk_hrpsys_ros_bridge`/models/staro.l -r ${ROBOT}

