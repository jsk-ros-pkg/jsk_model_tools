macro(get_simtrans_exe _simtrans_exe)
  find_program(${_simtrans_exe} simtrans)
  if("${${_simtrans_exe}}$" STREQUAL "${_simtrans_exe}-NOTFOUND")
    message(WARNING "Could not find simtrans. Please install simtrans from https://github.com/fkanehiro/simtrans to convert models")
    set(${_simtrans_exe} "")
  endif()
  message(STATUS "Found simtrans: ${${_simtrans_exe}}")
endmacro(get_simtrans_exe _simtrans_exe)

macro(get_meshlabserver_exe _meshlabserver_exe)
  find_program(${_meshlabserver_exe} meshlabserver)
  if("${${_meshlabserver_exe}}$" STREQUAL "${_meshlabserver_exe}-NOTFOUND")
    message(FATAL_ERROR "Could not find meshlabserver.")
  endif()
  message(STATUS "Found meshlabserver: ${${_meshlabserver_exe}}")
endmacro(get_meshlabserver_exe _meshlabserver_exe)

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
      COMMAND python
      ARGS ${PROJECT_SOURCE_DIR}/scripts/convertworld.py ${GAZEBOWORLD_FILE} ${PROJECT_SOURCE_DIR}/worlds/${GAZEBO_WORLD_NAME}.yaml
      DEPENDS ${PROJECT_SOURCE_DIR}/scripts/convertworld.py ${GAZEBOWORLD_FILE}
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
    list(APPEND WORLD_FILES "${PROJECT_SOURCE_DIR}/worlds/${GAZEBO_WORLD_NAME}.yaml")

  endforeach(GAZEBOWORLD_FILE)
  add_custom_target(eusurdfwrl_environment_yamls ALL DEPENDS ${WORLD_FILES})
endfunction(convert_gazebo_world_to_environment_yaml)

function(convert_urdf_to_wrl)
  get_eusurdfdir(EUSURDFDIR)
  get_simtrans_exe(SIMTRANS_EXE)
  get_meshlabserver_exe(MESHLABSERVER_EXE)

  if(EXISTS "${SIMTRANS_EXE}")
    set(VRML_FILES "")
    set(TMP_VRML_FILES "")
    file(GLOB URDF_FILES "${EUSURDFDIR}/models/*/model.urdf")
    set(GAZEBO_MODEL_PATH "${EUSURDFDIR}/models:$ENV{GAZEBO_MODEL_PATH}")
    foreach(URDF_FILE ${URDF_FILES})
      string(REGEX REPLACE "^.*/models/(.*)/model.urdf$" "\\1"
        URDF_NAME ${URDF_FILE})
      set(MODEL_OUT_DIR ${PROJECT_SOURCE_DIR}/models)
      set(TMP_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/models/${URDF_NAME})
      if(NOT EXISTS "${TMP_OUT_DIR}")
        file(MAKE_DIRECTORY "${TMP_OUT_DIR}")
      endif()

      # generate model
      add_custom_command(
        OUTPUT ${TMP_OUT_DIR}/${URDF_NAME}.wrl
        # -b : use both visual and collision
        # -e 100: estimate mass and inertia from bounding box. (kg/m^3)
        COMMAND GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH} ${SIMTRANS_EXE} -b -e 100 -i ${URDF_FILE} -o ${TMP_OUT_DIR}/${URDF_NAME}.wrl
        DEPENDS ${URDF_FILE}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
      list(APPEND TMP_VRML_FILES "${TMP_OUT_DIR}/${URDF_NAME}.wrl")

      add_custom_command(
        OUTPUT ${MODEL_OUT_DIR}/${URDF_NAME}.wrl
        COMMAND ls ${TMP_OUT_DIR} | grep collision.wrl | xargs -P 1 -I@ ${MESHLABSERVER_EXE} -i ${TMP_OUT_DIR}/@ -o ${TMP_OUT_DIR}/@ -s ${PROJECT_SOURCE_DIR}/scripts/filter.mlx
        COMMAND ls ${TMP_OUT_DIR} | xargs -P 1 -I@ cp ${TMP_OUT_DIR}/@ ${MODEL_OUT_DIR}/
        DEPENDS ${TMP_OUT_DIR}/${URDF_NAME}.wrl
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
      list(APPEND VRML_FILES "${MODEL_OUT_DIR}/${URDF_NAME}.wrl")

    endforeach(URDF_FILE)
    add_custom_target(eusurdfwrl_models ALL DEPENDS ${VRML_FILES})
  endif()
endfunction(convert_urdf_to_wrl)
