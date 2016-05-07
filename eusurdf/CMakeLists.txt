cmake_minimum_required(VERSION 2.8.3)
project(eusurdf)

find_package(catkin REQUIRED)
find_package(collada_urdf_jsk_patch)

catkin_package()

# get path to collada_to_urdf
macro(get_collada_to_urdf _collada_to_urdf_exe)
  find_package(collada_urdf_jsk_patch REQUIRED)
  set(${_collada_to_urdf_exe} ${collada_urdf_jsk_patch_PREFIX}/lib/collada_urdf_jsk_patch/collada_to_urdf)
  if(NOT EXISTS ${${_collada_to_urdf_exe}})
    message(FATAL_ERROR "could not find ${${_collada_to_urdf_exe}}")
  endif()
endmacro(get_collada_to_urdf _collada_to_urdf_exe)

set(_generated_files ${PROJECT_SOURCE_DIR}/models/GENERATED)
get_collada_to_urdf(_collada_to_urdf_exe)

add_custom_command(
  OUTPUT ${_generated_files}
  COMMAND "${PROJECT_SOURCE_DIR}/scripts/convert-scene-to-urdf.py" ARGS ${PROJECT_SOURCE_DIR} ${_collada_to_urdf_exe}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

add_custom_target(eusurdf_install ALL DEPENDS ${_generated_files})

install(DIRECTORY models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/convert-eus-to-urdf-test.launch)
endif()
