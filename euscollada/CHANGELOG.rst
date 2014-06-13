^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package euscollada
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* revert codes for collision model making according to https://github.com/euslisp/jskeus/pull/93 and https://github.com/jsk-ros-pkg/jsk_model_tools/pull/46
* Enable euscollada conversion test ;; Add dependency on pr2_mechanism_model to travis.yaml ;; Fix cmake and use unittest.l in pr2.sh to trap Euslisp error
* (https://github.com/jsk-ros-pkg/jsk_model_tools/issues/18) euscollada/src/collada2eus_urdfmodel.cpp : do not overwrite sensor methods
* (jsk-ros-pkg/jsk_model_tools/issues/18) euscollada/src/collada2eus.cpp : do not overwrite sensors methods ;; sensors method are supported from euslisp/jskeus/pull/92
* (jsk-ros-pkg/jsk_model_tools/issues/41) euscollada/src/euscollada-robot*.l : move collision model codes to irtrobot.l https://github.com/euslisp/jskeus/pull/93
* (jsk-ros-pkg/jsk_model_tools/issues/18) euscollada/src/euscollada*.l : remove deprecate sensor methods ;; latest sensor methods are added and testes by https://github.com/euslisp/jskeus/pull/92
* fix sensor coords
* Contributors: Yohei Kakiuchi, Shunichi Nozawa

0.1.3 (2014-05-01)
------------------
* Merge pull request `#35 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/35>`_ from k-okada/add_tf_depends
  add tf to depend
* Contributors: Kei Okada

0.1.2 (2014-05-01)
------------------
* put catkin_package after find_package(catkin)
* Contributors: Kei Okada

0.1.1 (2014-05-01)
------------------
* check if pr2_mechanism_model exists
* add rosboost_cfg, qhull and cmake_modules to depends
* use assimp_devel pkgconfig
* (euscollada) update for assimp_devel in jsk_common (`#20 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/20>`_)
* support string argument for sensor accessor methods discussed in https://github.com/jsk-ros-pkg/jsk_model_tools/issues/18
* add rosbduil/mk to depend
* remove denepends to jsk_tools whcih is used for launch doc
* add add_dependancies
* remove urdf_parser, it is included in urdfdom
* add making collada2eus_urdfmodel in catkin
* udpate euscollada for groovy
* update manifest at euscollada
* remove debug message
* fix make pr2 instance if *pr2* does not exists
* do not use glvertices on collada-body if it does not exists
* fix using non-existing tag/body
* `#2 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/2>`_: omit ik demo
* `#2 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/2>`_: omit PR2 IK test from euscollada to avoid intermediate dependency
* `#2 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/2>`_: add yaml-cpp to euscollada dependency
* sorting sensor order of urdfmodel
* add small cube if geometry does not exist
* add comment for using assimp_devel
* add some scripts for fixing collada error
* add printing sensor methods to euscollada_urdf
* add euscollada-robot_urdfmodel.l
* revert euscollada-robot.l
* update mesh post process
* fix minor bug
* update collada2eus_urdfmodel
* install src directory in euscollada because euscollada-robot.l is in src
* install collada2eus
* fix link association and material on collada2eus_urdfmodel.cpp
* update collada2eus_urdfmodel.cpp
* update collada2eus_urdfmodel.cpp
* add rosdep collada_urdf for rosdep install
* update collada2eus_urdfmodel.cpp
* change description in euscollada-robot.l
* small update
* remove compile test program
* add dependancy for assimp
* add collada2eus_urdfmodel, but it is not working well now
* add collada2eus for using urdfmodel
* dump sensor name as string instead of using symbol with colon to keep lower-case and upper-case
* add writeNodeMassFrames function ;; write node MassFrame regardless of geometory existence
* fix parenthesis of bodyset-link definition ;; separate mass frame writing
* find thisArticulated which has extra array
* append additional-weight-list
* use additional-weight-list instead of weight
* separate defining of sensor name method
* catkinze euscollada
* fix bug discussioned in [`#243 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/243>`_]
* add for reading <actuator> <nominal_torque>
* add :max-joint-torque
* move collada-body definition to euscollada-robot.l
* add checking body has glvertices
* fix typo in :init-ending
* add make-detail-collision-model-from-glvertices-for-one-link
* use transform from associated parent link
* add name to end-coords
* enable to generate and display models which bodies have no vertices
* fix - -> _ for bodies name
* add robot_name to link body
* use :links to obtain sensor's parent link
* create output(lisp) file after successfully parsed collada file, see https://code.google.com/p/rtm-ros-robotics/issues/detail?id=164
* add use_speed_limit parameter to collada2eus for avoiding to use speed-limit
* fix matrix multiple bug for inertia tensor, [`#222 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/222>`_]
* modify precision for printing euslisp model file, [`#222 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/222>`_]
* add target for conversion from irteus to collada ;; does not add this conversion to default ALL target
* use collad_directory for irteus -> collada output directory
* remove test code depends on glc-capture
* add barrett-wam and debug message
* add barrett test
* comment out warning message
* do not support non-sensor keyword method
* link's instance name have _lk suffix, buf link's name itself does not have suffix, [`#200 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/200>`_]
* update: nao.sh
* fix: joint-angle on nao.yaml
* add add_joint_suffix and set add_link_suffix and add_joint_suffix as default
* add accessor by limb name
* fix :set-color method of collada-body
* add dump of imu sensor and imusensor methods
* add :set-color method for overwrighting geometry color
* add --add-link-suffix option to collada2eus for avoiding to add the same name to link and joint
* move collada2eus_dev.cpp to collada2eus.cpp
* move collada2eus.cpp to collada2eus_old.cpp
* fix: parsing transformation in conllada file (experimental)
* revert [`#1445 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/1445>`_], since min/max limit of infinite rotational joint has changed from 180 to 270 in https://sourceforge.net/p/jskeus/tickets/25/
* set recommended stop and cog-gain param
* overwrite fullbody-inverse-kinematics method ;; test on euscollada-robot
* switch collada2eus to use glvertices for visualization
* fix wreit-r of reset pose from 180->0 [`#145 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/145>`_]
* add dom like function to using sxml
* update index.rst,conf.py by Jenkins
* update index.rst,conf.py by Jenkins
* update index.rst,conf.py by Jenkins
* use collada_urdf instead of collada_urdf_jsk_patch, jsk_patch is subitted to upstream see https://github.com/ros/robot_model/pull/15/
* update index.rst,conf.py by Jenkins
* update index.rst,conf.py by Jenkins
* merge updates on collada2eus.cpp
* merge updates on collada2eus.cpp
* remove unused string
* find root-link by tracing limb's link list
* use robot_name instead of thisNode->getName
* add robotname to body classes to avoid duplicate naming
* add comment for mass property fix ;; add sensor calling method according to pr2eus/pr2.l's :camera method
* add getSensorType for attach_sensor
* add force-sensors from attached sensor according to pr2eus/pr2.l's :cameras method
* add attach_sensor coords method
* fix bug of mass_frame interpretation ;; support multiple mass_frame description (e.g., VRML->collada file) ;; tempolariry calculate link-local mass property in euscollada-robot's :init-ending
* fix for converting multiple meshe groups
* add collada2eus_dev for development version using glvertices
* fix bug in manipulator's make-coords ;; :axis must non-zero vector ;; some codes about :axis should be fixed
* fix for groovy
* fix for groovy, not using new DAE()
* move rosdep from euscollada to jsk_model_tools since due to package euscollada being in a satck
* update index.rst,conf.py by Jenkins
* add eus_assimp for eusing assimp library on EusLisp
* move euscollada,collada_tools,assimp_devl to jsk_model_tools
* Contributors: Kei Okada, Ryohei Ueda, YoheiKakiuchi, Masaki Murooka, Shunnichi Nozawa
