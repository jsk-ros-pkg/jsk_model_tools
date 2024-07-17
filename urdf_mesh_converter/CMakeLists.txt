cmake_minimum_required(VERSION 2.8.3)
project(urdf_mesh_converter)

Find_package(
  catkin REQUIRED
  catkin_virtualenv
)

if (${catkin_virtualenv_VERSION} VERSION_LESS "0.5.1")
  message(WARNING "${PROJECT_NAME} requires catkin_virtualenv >= 0.5.1")
  return()
endif()


catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
)

if($ENV{ROS_DISTRO} STREQUAL "noetic")
  catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.in
    PYTHON_INTERPRETER python3
  )
else()
  message(WARNING "This package uses trimesh for model conversion. Trimesh has discontinued support for Python 2 quite some time ago. Please use Python 3 or later.")
  return()
endif()

file(GLOB SCRIPTS_FILES scripts/*)
catkin_install_python(
  PROGRAMS ${SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)

  set(python_test_scripts
    tests/test_urdf_mesh_converter.py
  )

  catkin_install_python(PROGRAMS ${python_test_scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  add_rostest(tests/test_urdf_mesh_converter.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv)
endif()


install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
