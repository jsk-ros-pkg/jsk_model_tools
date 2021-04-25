macro(get_simtrans_exe _simtrans_exe)
  find_program(${_simtrans_exe} simtrans)
  if("${${_simtrans_exe}}$" STREQUAL "${_simtrans_exe}-NOTFOUND")
    message(WARNING "Could not find simtrans. Please install simtrans from https://github.com/fkanehiro/simtrans to convert models")
    set(${_simtrans_exe} "")
  endif()
  message(STATUS "Found simtrans: ${${_simtrans_exe}}")
endmacro(get_simtrans_exe _simtrans_exe)

macro(get_eusurdfdir _eusurdfdir_var)
  find_package(eusurdf REQUIRED)
  if(EXISTS ${eusurdf_SOURCE_PREFIX})
    set(${_eusurdfdir_var} "${eusurdf_SOURCE_PREFIX}")
  else()
    set(${_eusurdfdir_var} "${eusurdf_PREFIX}/share/eusurdf")
  endif()
  message(STATUS "Found eusurdf dir: ${${_eusurdfdir_var}}")
endmacro(get_eusurdfdir _eusurdfdir_var)

function(convert_gazebo_world_to_environment_yaml)
  get_eusurdfdir(EUSURDFDIR)
  set(WORLD_FILES "")
  file(GLOB GAZEBOWORLD_FILES "${EUSURDFDIR}/worlds/*.world")
  foreach(GAZEBOWORLD_FILE ${GAZEBOWORLD_FILES})
    string(REGEX REPLACE "^.*/(.*).world$" "\\1"
      GAZEBO_WORLD_NAME ${GAZEBOWORLD_FILE})

    add_custom_command(
      OUTPUT ${PROJECT_SOURCE_DIR}/worlds/${GAZEBO_WORLD_NAME}.yaml
      # convertworld.py requires virtualenv
      COMMAND ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/convertworld.py ${GAZEBOWORLD_FILE} ${PROJECT_SOURCE_DIR}/worlds/${GAZEBO_WORLD_NAME}.yaml
      DEPENDS ${PROJECT_SOURCE_DIR}/scripts/convertworld.py ${GAZEBOWORLD_FILE} ${PROJECT_NAME}_generate_virtualenv
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
    list(APPEND WORLD_FILES "${PROJECT_SOURCE_DIR}/worlds/${GAZEBO_WORLD_NAME}.yaml")

  endforeach(GAZEBOWORLD_FILE)
  add_custom_target(eusurdfwrl_environment_yamls ALL DEPENDS ${WORLD_FILES})
endfunction(convert_gazebo_world_to_environment_yaml)

function(convert_urdf_to_wrl)
  get_eusurdfdir(EUSURDFDIR)
  get_simtrans_exe(SIMTRANS_EXE)

  if(EXISTS "${SIMTRANS_EXE}")
    set(VRML_FILES "")
    file(GLOB URDF_FILES "${EUSURDFDIR}/models/*/model.urdf")
    set(GAZEBO_MODEL_PATH "${EUSURDFDIR}/models:$ENV{GAZEBO_MODEL_PATH}")
    foreach(URDF_FILE ${URDF_FILES})
      string(REGEX REPLACE "^.*/models/(.*)/model.urdf$" "\\1"
        URDF_NAME ${URDF_FILE})

      # generate model
      set(MODEL_OUT_DIR ${PROJECT_SOURCE_DIR}/models/${URDF_NAME})
      if(NOT EXISTS "${MODEL_OUT_DIR}")
        file(MAKE_DIRECTORY "${MODEL_OUT_DIR}")
      endif()
      add_custom_command(
        OUTPUT ${MODEL_OUT_DIR}/${URDF_NAME}.wrl
        # -b : use both visual and collision
        # -e 100: estimate mass and inertia from bounding box. (kg/m^3)
        COMMAND GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH} ${SIMTRANS_EXE} -b -e 100 -i ${URDF_FILE} -o ${MODEL_OUT_DIR}/${URDF_NAME}.wrl
        DEPENDS ${URDF_FILE}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
      list(APPEND VRML_FILES "${MODEL_OUT_DIR}/${URDF_NAME}.wrl")

    endforeach(URDF_FILE)
    add_custom_target(eusurdfwrl_models ALL DEPENDS ${VRML_FILES})
  endif()
endfunction(convert_urdf_to_wrl)
