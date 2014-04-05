cmake_minimum_required(VERSION 2.8.3)
project(collada_tools)

if($ENV{ROS_DISTRO} STREQUAL "groovy")
  find_package(catkin REQUIRED COMPONENTS roscpp urdf_parser assimp_devel urdf collada_parser)
else()
  find_package(catkin REQUIRED COMPONENTS roscpp urdf_parser_plugin assimp_devel urdf collada_parser)
endif()
find_package(Boost REQUIRED COMPONENTS filesystem program_options)

#set(CMAKE_MODULE_PATH  ${PROJECT_SOURCE_DIR}/cmake-extensions/ )
find_package(PkgConfig)
pkg_check_modules(COLLADADOM collada-dom-150)
include_directories(${COLLADADOM_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${COLLADADOM_LIBRARY_DIRS})

# check_function_exists(mkstemps HAVE_MKSTEMPS)
# if( HAVE_MKSTEMPS )
#   add_definitions("-DHAVE_MKSTEMPS")
# endif()

catkin_package()

set(SOURCE_FILES
  src/collada_gazebo_gen.cpp
  src/collada_to_graphviz.cpp
  src/collada_to_urdf.cpp
  )

add_executable(collada_gazebo_gen src/collada_gazebo_gen.cpp)
set_target_properties(collada_gazebo_gen PROPERTIES LINK_FLAGS "-Wl,--no-as-needed")
target_link_libraries(collada_gazebo_gen ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(collada_to_graphviz src/collada_to_graphviz.cpp)
set_target_properties(collada_to_graphviz PROPERTIES LINK_FLAGS "-Wl,--no-as-needed")
target_link_libraries(collada_to_graphviz ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(collada_to_urdf src/collada_to_urdf.cpp)
set_target_properties(collada_to_urdf PROPERTIES LINK_FLAGS "-Wl,--no-as-needed")
target_link_libraries(collada_to_urdf ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS collada_to_urdf collada_to_graphviz collada_gazebo_gen
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
