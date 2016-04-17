# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(euscollada)

# check rosversion if >=indigo
execute_process(
  COMMAND python -c "import sys; sys.exit(ord('$ENV{ROS_DISTRO}'[0]) >= ord('indigo'[0]))"
  RESULT_VARIABLE rosversion_ge_indigo)

IF(${rosversion_ge_indigo})
find_package(catkin REQUIRED COMPONENTS collada_urdf rospack collada_parser resource_retriever pr2_description)
ELSE()
find_package(catkin REQUIRED COMPONENTS collada_urdf rospack collada_parser urdfdom resource_retriever)
ENDIF()

catkin_package()

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")

find_package(PkgConfig)
IF(${rosversion_ge_indigo})
pkg_check_modules(urdf_dom urdfdom REQUIRED)
ENDIF()
pkg_check_modules(colladadom collada-dom-150 REQUIRED)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)
IF(${yaml_cpp_VERSION} VERSION_GREATER "0.5.0")
## indigo yaml-cpp : 0.5.0 /  hydro yaml-cpp : 0.3.0
  add_definitions("-DUSE_CURRENT_YAML")
ENDIF()
pkg_check_modules(assimpdevel assimp_devel REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${colladadom_INCLUDE_DIRS} ${yaml_cpp_INCLUDE_DIRS} ${assimpdevel_INCLUDE_DIRS} ${urdf_dom_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${assimpdevel_LIBRARY_DIRS})

add_executable(collada2eus src/collada2eus.cpp)
target_link_libraries(collada2eus ${catkin_LIBRARIES} qhull ${yaml_cpp_LIBRARIES} ${colladadom_LIBRARIES} ${recource_retriever_LIBRARIES})
add_dependencies(collada2eus libassimp_devel)

find_package(Boost REQUIRED system filesystem)
include_directories(${Boost_INCLUDE_DIR})
add_executable(collada2eus_urdfmodel src/collada2eus_urdfmodel.cpp)
target_link_libraries(collada2eus_urdfmodel ${catkin_LIBRARIES} qhull ${yaml_cpp_LIBRARIES} ${colladadom_LIBRARIES} ${collada_parser_LIBRARIES} ${recource_retriever_LIBRARIES} ${assimpdevel_LIBRARIES} ${Boost_LIBRARIES} ${urdf_dom_LIBRARIES})
add_dependencies(collada2eus_urdfmodel libassimp_devel)

install(TARGETS collada2eus collada2eus_urdfmodel
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY src scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN "*.cpp" EXCLUDE)


file(GLOB _install_files RELATIVE ${PROJECT_SOURCE_DIR} *.yaml *.sh)
install(FILES ${_install_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


# --------------
# Generate pr2.l
# --------------
if(EXISTS ${pr2_description_PREFIX}/share/pr2_description/robots/pr2.urdf.xacro)
  set(pr2_xacro ${pr2_description_PREFIX}/share/pr2_description/robots/pr2.urdf.xacro)
elseif(EXISTS ${pr2_mechanism_model_SOURCE_PREFIX}/robots/pr2.urdf)
  set(pr2_xacro ${pr2_description_SOURCE_PREFIX}/robots/pr2.urdf.xacro)
else()
  message(WARNING "Could not found pr2.urdf.xacro in ${pr2_description_PREFIX}/share/pr2_description/robots/pr2.urdf.xacro and ${pr2_description_SOURCE_PREFIX}/robots/pr2.urdf.xacro")
endif()
if (EXISTS ${pr2_xacro})
  message(STATUS "Found pr2.urdf.xacro at ${pr2_xacro}")
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/pr2.urdf
    COMMAND rosrun xacro xacro ${pr2_xacro} > pr2.urdf
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS ${pr2_xacro})
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/pr2.dae
    COMMAND rosrun collada_urdf urdf_to_collada pr2.urdf pr2.dae
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS pr2.urdf)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/pr2.l
    COMMAND rosrun euscollada collada2eus pr2.dae pr2.yaml pr2.l
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS pr2.dae pr2.yaml)
  # get collada-dom version
  find_package(PkgConfig)
  pkg_check_modules(COLLADA collada-dom>=2.4.4 )
  if ( ${COLLADA_FOUND} )
    add_custom_target(generate_pr2_lisp ALL DEPENDS ${PROJECT_SOURCE_DIR}/pr2.l)
  else()
    pkg_check_modules(COLLADA collada-dom)
    message(WARNING "urdf_to_collada requries collada-dom >= 2.4.4, installed version is ${COLLADA_VERSION}")
  endif()
endif()
