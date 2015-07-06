cmake_minimum_required(VERSION 2.8.3)
project(eusurdf)

find_package(catkin REQUIRED)

catkin_package()

set(_generated_files ${PROJECT_SOURCE_DIR}/models/GENERATED)

add_custom_command(
  OUTPUT ${_generated_files}
  COMMAND "${PROJECT_SOURCE_DIR}/scripts/convert-scene-to-urdf.py"
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

add_custom_target(eusurdf_install ALL DEPENDS ${_generated_files})

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
