# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(euscollada)

IF($ENV{ROS_DISTRO} STREQUAL "indigo")
find_package(catkin REQUIRED COMPONENTS collada_urdf rospack collada_parser resource_retriever)
ELSE()
find_package(catkin REQUIRED COMPONENTS collada_urdf rospack collada_parser urdfdom resource_retriever)
ENDIF()

catkin_package()

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")

find_package(PkgConfig)
IF($ENV{ROS_DISTRO} STREQUAL "indigo")
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

find_package(Boost REQUIRED system)
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

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_custom_command(OUTPUT sample1.dae
    COMMAND apt-get download ros-$ENV{ROS_DISTRO}-openhrp3
    COMMAND cmake -E make_directory ./opt/ros/$ENV{ROS_DISTRO}/share/OpenHRP-3.1/sample/model/
    COMMAND dpkg -x ros-$ENV{ROS_DISTRO}-openhrp3*.deb .
    COMMAND cp ./opt/ros/$ENV{ROS_DISTRO}/share/OpenHRP-3.1/sample/model/sample1.dae sample1.dae
    DEPENDS collada2eus)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/test/sample1-noyaml.l
    COMMAND rosrun euscollada collada2eus sample1.dae ${PROJECT_SOURCE_DIR}/test/sample1-noyaml.l
    DEPENDS sample1.dae ${PROJECT_SOURCE_DIR}/test/sample1.yaml)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/test/sample1-yaml.l
    COMMAND rosrun euscollada collada2eus sample1.dae ${PROJECT_SOURCE_DIR}/test/sample1.yaml ${PROJECT_SOURCE_DIR}/test/sample1-yaml.l
    DEPENDS sample1.dae ${PROJECT_SOURCE_DIR}/test/sample1.yaml)
  add_custom_target(generate_test_files ALL DEPENDS ${PROJECT_SOURCE_DIR}/test/sample1-noyaml.l ${PROJECT_SOURCE_DIR}/test/sample1-yaml.l)
  add_rostest(test/test-sample-robot.test)
endif()
