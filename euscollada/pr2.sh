#!/bin/bash

cd `rospack find euscollada`

rosrun collada_urdf_jsk_patch urdf_to_collada `rospack find pr2_mechanism_model`/pr2.urdf pr2.dae
if [ "$?" != 0 ] ;  then exit ; fi

rosrun euscollada collada2eus pr2.dae pr2.yaml pr2.l.$$.tmp; mv pr2.l.$$.tmp pr2.l
if [ "$?" != 0 ] ;  then exit ; fi

rosrun roseus roseus "\
(progn									\
  (load \"package://euscollada/pr2.l\")					\
  (load \"package://pr2eus/pr2-utils.l\")				\
  (if (and x::*display* (> x::*display* 0) (not (boundp '*irtviewer*))) (make-irtviewer :title \"pr2.sh\"))			\
  (if (not (boundp '*pr2*)) (pr2))					\
									\
  (send *pr2* :move-to (make-coords) :world)				\
  (send *pr2* :reset-pose)						\
  (when (boundp '*irtviewer*) (objects (list *pr2*)))			\
  (setq i 0)								\
  (do-until-key								\
   (print (list i (send *pr2* :torso :waist-z :joint-angle)))		\
   (setq p (make-coords :pos						\
                        (v+ (float-vector 400 -400 1000)		\
                            (float-vector				\
                             0						\
                             (* 300 (sin (* pi (/ i 25.0))))		\
                             (* 500 (sin (* pi (/ i 50.0))))))))	\
   (send *pr2* :inverse-kinematics p					\
         :look-at-target t						\
         :rotation-axis :z						\
         :use-base 0.1							\
         )								\
   (when (boundp '*irtviewer*)						\
	(send *irtviewer* :objects (list *pr2* p))			\
	(send *irtviewer* :draw-objects)				\
	(x::window-main-one))						\
   (incf i)								\
   (when (> i 100) (exit 0))						\
   )									\
  (if (boundp '*irtviewer*) (send-all (send *pr2* :links) :draw-on :flush t))\
  (exit)                                                                \
  )									\
"
