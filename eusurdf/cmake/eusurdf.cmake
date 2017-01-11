macro(get_collada_to_urdf_exe _collada_to_urdf_exe)
  find_package(collada_urdf_jsk_patch REQUIRED)
  set(${_collada_to_urdf_exe} ${collada_urdf_jsk_patch_PREFIX}/lib/collada_urdf_jsk_patch/collada_to_urdf)
  if(NOT EXISTS ${${_collada_to_urdf_exe}})
    message(FATAL_ERROR "could not find ${${_collada_to_urdf_exe}}")
  endif()
  message(STATUS "Found collada_to_urdf: ${${_collada_to_urdf_exe}}")
endmacro(get_collada_to_urdf_exe _collada_to_urdf_exe)

macro(get_eusdir _eusdir_var)
  find_package(euslisp QUIET)
  if(euslisp_FOUND)
    set(${_eusdir_var} "${EUSDIR}")
  elseif(NOT "$ENV{EUSDIR}" STREQUAL "")
    set(${_eusdir_var} "$ENV{EUSDIR}")
  else(euslisp_FOUND)
    message(FATAL_ERROR "EUSDIR could not detected.")
  endif(euslisp_FOUND)
  message(STATUS "Found EUSDIR: ${${_eusdir_var}}")
endmacro(get_eusdir _eusdir_var)

macro(get_eusexe _eus_exe)
  find_package(euslisp REQUIRED)
  if(${CMAKE_SYSTEM_NAME} MATCHES Linux)
    execute_process(COMMAND gcc -dumpmachine OUTPUT_VARIABLE GCC_MACHINE OUTPUT_STRIP_TRAILING_WHITESPACE)
    message("-- Set GCC_MACHINE to ${GCC_MACHINE}")
    if(${GCC_MACHINE} MATCHES x86_64-linux-gnu)
      set(ARCHDIR Linux64)
    elseif(${GCC_MACHINE} MATCHES i686-linux-gnu)
      set(ARCHDIR Linux)
    elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR
	${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* )
      set(ARCHDIR Linux64)
    else()
      set(ARCHDIR Linux)
    endif()
  elseif(${CMAKE_SYSTEM_NAME} MATCHES Darwin)
    set(ARCHDIR Darwin)
  elseif(${CMAKE_SYSTEM_NAME} MATCHES Cygwin)
    set(ARCHDIR Cygwin)
  else()
    set(ARCHDIR Generic)
  endif()

  if (EXISTS ${euslisp_SOURCE_DIR}/jskeus/eus/${ARCHDIR}/bin/irteusgl)
    set(${_eus_exe} ${euslisp_SOURCE_DIR}/jskeus/eus/${ARCHDIR}/bin/irteusgl)
  elseif (EXISTS ${euslisp_SOURCE_PREFIX}/jskeus/eus/${ARCHDIR}/bin/irteusgl)
    set(${_eus_exe} ${euslisp_SOURCE_PREFIX}/jskeus/eus/${ARCHDIR}/bin/irteusgl)
  elseif (EXISTS ${euslisp_PREFIX}/share/euslisp/jskeus/eus/${ARCHDIR}/bin/irteusgl)
    set(${_eus_exe} ${euslisp_PREFIX}/share/euslisp/jskeus/eus/${ARCHDIR}/bin/irteusgl)
  else (EXISTS ${euslisp_SOURCE_DIR}/jskeus/eus/${ARCHDIR}/bin/irteusgl)
    message(FATAL_ERROR "cannot find irteusgl")
  endif(EXISTS ${euslisp_SOURCE_DIR}/jskeus/eus/${ARCHDIR}/bin/irteusgl)
  message(STATUS "Found eus executable: ${${_eus_exe}}")
endmacro(get_eusexe _eusexe)

function(convert_eusscene_to_gazebo_world)
  get_eusdir(EUSDIR)
  get_eusexe(EUS_EXE)
  set(WORLD_FILES "")
  file(GLOB EUSSCENE_FILES "${EUSDIR}/models/*-scene.l")
  foreach(EUSSCENE_FILE ${EUSSCENE_FILES})
    string(REGEX REPLACE "^.*/(.*)-scene.l$" "\\1"
      EUS_SCENE_NAME ${EUSSCENE_FILE})

    add_custom_command(
      OUTPUT ${PROJECT_SOURCE_DIR}/worlds/${EUS_SCENE_NAME}.world
      COMMAND ${EUS_EXE}
      ARGS ${PROJECT_SOURCE_DIR}/euslisp/eusscene_to_world.l ${EUSSCENE_FILE} ${PROJECT_SOURCE_DIR}/worlds/${EUS_SCENE_NAME}.world
      MAIN_DEPENDENCY ${PROJECT_SOURCE_DIR}/euslisp/eusscene_to_world.l
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
    list(APPEND WORLD_FILES "${PROJECT_SOURCE_DIR}/worlds/${EUS_SCENE_NAME}.world")

  endforeach(EUSSCENE_FILE)
  add_custom_target(eusurdf_scene_worlds ALL DEPENDS ${WORLD_FILES})
endfunction(convert_eusscene_to_gazebo_world)

function(convert_eusscene_to_urdf_xacro)
  get_eusdir(EUSDIR)
  get_eusexe(EUS_EXE)
  set(XACRO_FILES "")
  file(GLOB EUSSCENE_FILES "${EUSDIR}/models/*-scene.l")
  foreach(EUSSCENE_FILE ${EUSSCENE_FILES})
    string(REGEX REPLACE "^.*/(.*)-scene.l$" "\\1"
      EUS_SCENE_NAME ${EUSSCENE_FILE})

    add_custom_command(
      OUTPUT ${PROJECT_SOURCE_DIR}/worlds/${EUS_SCENE_NAME}.urdf.xacro
      COMMAND ${EUS_EXE}
      ARGS ${PROJECT_SOURCE_DIR}/euslisp/eusscene_to_xacro.l ${EUSSCENE_FILE} ${PROJECT_SOURCE_DIR}/worlds/${EUS_SCENE_NAME}.urdf.xacro
      MAIN_DEPENDENCY ${PROJECT_SOURCE_DIR}/euslisp/eusscene_to_xacro.l
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
    list(APPEND XACRO_FILES "${PROJECT_SOURCE_DIR}/worlds/${EUS_SCENE_NAME}.urdf.xacro")
  endforeach(EUSSCENE_FILE)
  add_custom_target(eusurdf_scene_xacros ALL DEPENDS ${XACRO_FILES})
endfunction(convert_eusscene_to_urdf_xacro)

function(convert_eusmodel_to_urdf)
  get_eusdir(EUSDIR)
  get_eusexe(EUS_EXE)
  get_collada_to_urdf_exe(COLLADA_URDF_EXE)
  set(URDF_FILES "")
  set(XACRO_FILES "")
  file(GLOB EUSMODEL_FILES "${EUSDIR}/models/*-object.l")
  foreach(EUSMODEL_FILE ${EUSMODEL_FILES})
    string(REGEX REPLACE "^.*/(.*)-object.l$" "\\1"
      EUS_MODEL_NAME ${EUSMODEL_FILE})
    set(MODEL_OUT_DIR ${PROJECT_SOURCE_DIR}/models/${EUS_MODEL_NAME})

    # generate model
    add_custom_command(
      OUTPUT ${MODEL_OUT_DIR}/model.urdf
      COMMAND ${EUS_EXE}
      ARGS ${PROJECT_SOURCE_DIR}/euslisp/eusmodel_to_urdf.l ${EUSMODEL_FILE} ${MODEL_OUT_DIR}/ ${COLLADA_URDF_EXE}
      MAIN_DEPENDENCY ${PROJECT_SOURCE_DIR}/euslisp/eusmodel_to_urdf.l
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
    list(APPEND URDF_FILES "${MODEL_OUT_DIR}/model.urdf")

    # generate static model
    add_custom_command(
      OUTPUT ${MODEL_OUT_DIR}_static/model.urdf
      COMMAND ${PROJECT_SOURCE_DIR}/scripts/make_static_model.py
      ARGS ${EUS_MODEL_NAME} ${PROJECT_SOURCE_DIR}
      MAIN_DEPENDENCY ${MODEL_OUT_DIR}/model.urdf)
    list(APPEND URDF_FILES "${MODEL_OUT_DIR}_static/model.urdf")

    # generate fixed model
    add_custom_command(
      OUTPUT ${MODEL_OUT_DIR}_fixed/model.urdf
      COMMAND ${PROJECT_SOURCE_DIR}/scripts/make_fixed_model.py
      ARGS ${EUS_MODEL_NAME} ${PROJECT_SOURCE_DIR}
      MAIN_DEPENDENCY ${MODEL_OUT_DIR}/model.urdf)
    list(APPEND URDF_FILES "${MODEL_OUT_DIR}_fixed/model.urdf")

    # generate xacro
    add_custom_command(
      OUTPUT ${MODEL_OUT_DIR}/model.urdf.xacro
      COMMAND ${PROJECT_SOURCE_DIR}/scripts/urdf_to_xacro.py
      ARGS -f package://eusurdf/models/ ${MODEL_OUT_DIR}/model.urdf ${MODEL_OUT_DIR}/model.urdf.xacro
      MAIN_DEPENDENCY ${MODEL_OUT_DIR}/model.urdf)
    list(APPEND XACRO_FILES "${MODEL_OUT_DIR}/model.urdf.xacro")
    add_custom_command(
      OUTPUT ${MODEL_OUT_DIR}_static/model.urdf.xacro
      COMMAND ${PROJECT_SOURCE_DIR}/scripts/urdf_to_xacro.py
      ARGS -f package://eusurdf/models/ ${MODEL_OUT_DIR}_static/model.urdf ${MODEL_OUT_DIR}_static/model.urdf.xacro
      MAIN_DEPENDENCY ${MODEL_OUT_DIR}_static/model.urdf)
    list(APPEND XACRO_FILES "${MODEL_OUT_DIR}_static/model.urdf.xacro")
    add_custom_command(
      OUTPUT ${MODEL_OUT_DIR}_fixed/model.urdf.xacro
      COMMAND ${PROJECT_SOURCE_DIR}/scripts/urdf_to_xacro.py
      ARGS -f package://eusurdf/models/ ${MODEL_OUT_DIR}_fixed/model.urdf ${MODEL_OUT_DIR}_fixed/model.urdf.xacro
      MAIN_DEPENDENCY ${MODEL_OUT_DIR}_fixed/model.urdf)
    list(APPEND XACRO_FILES "${MODEL_OUT_DIR}_fixed/model.urdf.xacro")
  endforeach(EUSMODEL_FILE)
  add_custom_target(eusurdf_models ALL DEPENDS ${URDF_FILES} ${XACRO_FILES})
endfunction(convert_eusmodel_to_urdf)
