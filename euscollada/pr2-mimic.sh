#!/bin/bash

if [ -w `rospack find euscollada` ]; then
    cd `rospack find euscollada`
else
    echo "euscollada direcotry is not writable, so write to current directory"
fi

#rosrun collada_urdf_jsk_patch urdf_to_collada `rospack find pr2_mechanism_model`/pr2.urdf pr2.dae
if [ -e `rospack find pr2_mechanism_model`/pr2.urdf ];
then
    rosrun collada_urdf urdf_to_collada `rospack find pr2_mechanism_model`/pr2.urdf pr2.dae
else
    rosrun xacro xacro.py `rospack find pr2_description`/robots/pr2.urdf.xacro > /tmp/pr2.$$.urdf
    rosrun collada_urdf urdf_to_collada /tmp/pr2.$$.urdf pr2.dae
fi
if [ "$?" != 0 ] ;  then exit ; fi

rosrun euscollada collada2eus pr2.dae `rospack find euscollada`/pr2-mimic.yaml pr2-mimic.l.$$.tmp; mv pr2-mimic.l.$$.tmp pr2-mimic.l
if [ "$?" != 0 ] ;  then exit ; fi

rosrun roseus roseus lib/llib/unittest.l "(init-unit-test)" "\
(progn									\
  (load \"pr2-mimic.l\")					\
  (if (and x::*display* (> x::*display* 0) (not (boundp '*irtviewer*))) (make-irtviewer :title \"pr2.sh\"))			\
  (if (not (boundp '*pr2*)) (pr2))					\
									\
  (when (boundp '*irtviewer*) (objects (list *pr2*)))			\
  (setq i 0.0)								\
  (do-until-key								\
   (send *pr2* :r_gripper :joint-angle i)				\
   (print (list i (send *pr2* :r_gripper_l_finger_joint :joint-angle)	\
              (send *pr2* :r_gripper_r_finger_joint :joint-angle)	\
              (send *pr2* :r_gripper_l_finger_tip_joint :joint-angle)	\
              (send *pr2* :r_gripper_r_finger_tip_joint :joint-angle)))	\
   (assert (eps= (send *pr2* :r_gripper_r_finger_joint :joint-angle)    \
                 (send *pr2* :r_gripper_l_finger_joint :joint-angle)))	\
   (assert (eps= (send *pr2* :r_gripper_r_finger_joint :joint-angle)    \
                 (send *pr2* :r_gripper_l_finger_tip_joint :joint-angle)))	\
   (assert (eps= (send *pr2* :r_gripper_r_finger_joint :joint-angle)    \
                 (send *pr2* :r_gripper_r_finger_tip_joint :joint-angle)))	\
   (when (boundp '*irtviewer*)						\
	(send *irtviewer* :draw-objects)				\
	)						\
   (incf i)								\
   (when (> i 30) (exit 0))						\
   )									\
  (if (boundp '*irtviewer*) (send-all (send *pr2* :links) :draw-on :flush t))\
  (exit)                                                                \
  )									\
"
