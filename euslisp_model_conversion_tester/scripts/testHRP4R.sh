#!/bin/bash

. $HOME/.bashrc

ROBOT=HRP4R
rosrun euslisp_model_conversion_tester testROBOT.sh -v $CVSDIR/OpenHRP/etc/$ROBOT/${ROBOT}main.wrl -r ${ROBOT}
