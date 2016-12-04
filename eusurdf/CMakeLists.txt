cmake_minimum_required(VERSION 2.8.3)
project(eusurdf)

find_package(catkin REQUIRED)
find_package(collada_urdf_jsk_patch)

catkin_package()

include(cmake/eusurdf.cmake)
convert_eusmodel_to_urdf()
convert_eusscene_to_gazebo_world()
convert_eusscene_to_urdf_xacro()

find_package(knowrob_srdl)
if(${knowrob_srdl_FOUND})
  convert_eusscene_to_srdl()
  install(DIRECTORY owl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
endif()

install(DIRECTORY models textured_models worlds launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/convert-eus-to-urdf-test.launch)
endif()
