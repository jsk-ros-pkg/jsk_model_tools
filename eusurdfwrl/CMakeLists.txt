cmake_minimum_required(VERSION 2.8.3)
project(eusurdfwrl)

find_package(catkin REQUIRED catkin_virtualenv)

catkin_package()

catkin_generate_virtualenv()

catkin_install_python(
  PROGRAMS scripts/convertworld.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include(cmake/eusurdfwrl.cmake)
convert_urdf_to_wrl()
convert_gazebo_world_to_environment_yaml()

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
