#!/bin/bash

. $HOME/.bashrc

ROBOT=SampleRobot
rosrun euslisp_model_conversion_tester testROBOT.sh -v `rospack find openhrp3`/share/OpenHRP-3.1/sample/model/sample1.wrl -r ${ROBOT}

