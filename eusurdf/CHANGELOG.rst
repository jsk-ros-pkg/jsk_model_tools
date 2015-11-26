^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package eusurdf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.14 (2015-11-26)
-------------------
* add instruction of converting eus->urdf
* modify sample usage comment of irteus2urdf-for-gazebo.
* use the resolved path of ros package to find eusurdf directory when path is nil. You can pass eusurdf path as argument to run in catkin build.
* add .gitignore to keep model directory
* generate model directory if not found.
* delete manifest.xml for gazebo model directory.
* Contributors: Masaki Murooka

0.1.13 (2015-09-01)
-------------------
* [eusurdf/package.xml] export gazebo_model_path for gazebo_ros
* - [eusurdf] remove rosbuild related scripts
  revert travis
* generate random tmp directory to avoid overwrite
* fix to use no rospack find nor rosrun for eusurdf
* convert models when catkin build
* add files to convert irtmodel to urdf
* delete converted urdf models in models directory.
* Contributors: Yuki Furuta, Masaki Murooka

0.1.12 (2015-05-07)
-------------------

0.1.11 (2015-04-09)
-------------------

0.1.10 (2015-04-02)
-------------------

0.1.9 (2015-04-01)
------------------

0.1.8 (2015-01-07)
------------------

0.1.7 (2014-12-19)
------------------
* Move scripts to euscollada to avoid catkinization of eusurdf
* Use link name, not joint name as parent link, but the solution is adhock
* add addLink function to add_sensor_to_urdf.py
* Add script to add end effector frames to urdf from yaml file for euslisp
* Add script to add sensor (fixed link) to urdf
* added moveit scene files
* add urdf models to eusurdf/models.
* Contributors: Ryohei Ueda, Masaki Murooka

0.1.6 (2014-06-30)
------------------

0.1.5 (2014-06-29)
------------------

0.1.4 (2014-06-15)
------------------

0.1.3 (2014-05-01 17:24)
------------------------

0.1.2 (2014-05-01 09:31)
------------------------

0.1.1 (2014-05-01 01:25)
------------------------
* set eusurdf and euslisp_model_conversion_tester to ROS_NOBUILD
* `#2 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/2>`_: make model directory before running xacro when building eusurdf
* fix hoge.stl->model.stl to pass hoge/fuga check
* update dirctory for xml2sxml
* use face-to-triangle-aux for triangulate faces
* update for using simple conversion
* fix for using package:// at inside jsk
* fix for using package:// at inside jsk
* fix, if link has no mesh
* fix checking which link has glvertices
* remove jsk internal dependancy
* add code for parsing inertial parameter
* debug for using fixed joint
* update for parsing sdf
* fix error message
* add heightmap tag to geometry/visual
* update for using :translate-vertices in eusurdf.l
* update for parsing cylinder and plane geometry
* update for using multi visual/geometry tags in link
* fix for parsing sdf file
* add eusurdf (copy from jsk-ros-pkg-unreleased)
* Contributors: Kei Okada, Ryohei Ueda, nozawa, youhei
