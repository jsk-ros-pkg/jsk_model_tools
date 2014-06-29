# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(euscollada)

find_package(catkin REQUIRED COMPONENTS collada_urdf rospack collada_parser urdfdom resource_retriever)

catkin_package()

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")

find_package(PkgConfig)
pkg_check_modules(colladadom collada-dom-150 REQUIRED)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)
pkg_check_modules(assimpdevel assimp_devel REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${colladadom_INCLUDE_DIRS} ${yaml_cpp_INCLUDE_DIRS} ${assimpdevel_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${assimpdevel_LIBRARY_DIRS})

add_executable(collada2eus src/collada2eus.cpp)
target_link_libraries(collada2eus ${catkin_LIBRARIES} qhull ${yaml_cpp_LIBRARIES} ${colladadom_LIBRARIES} ${recource_retriever_LIBRARIES})
add_dependencies(collada2eus libassimp_devel)

find_package(Boost REQUIRED system)
include_directories(${Boost_INCLUDE_DIR})
add_executable(collada2eus_urdfmodel src/collada2eus_urdfmodel.cpp)
target_link_libraries(collada2eus_urdfmodel ${catkin_LIBRARIES} qhull ${yaml_cpp_LIBRARIES} ${colladadom_LIBRARIES} ${collada_parser_LIBRARIES} ${recource_retriever_LIBRARIES} ${assimpdevel_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(collada2eus_urdfmodel libassimp_devel)

install(TARGETS collada2eus collada2eus_urdfmodel
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)


file(GLOB _install_files RELATIVE ${PROJECT_SOURCE_DIR} *.yaml *.sh)
install(FILES ${_install_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
