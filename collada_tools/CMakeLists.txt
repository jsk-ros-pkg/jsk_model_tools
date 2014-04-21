if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(CMAKE_MODULE_PATH  ${PROJECT_SOURCE_DIR}/cmake-extensions/ )
find_package(PkgConfig)
pkg_check_modules(COLLADADOM collada15dom)
include_directories(${COLLADADOM_INCLUDE_DIRS})
link_directories(${COLLADADOM_LIBRARY_DIRS})

rosbuild_find_ros_package(assimp_devel)
set(ENV{PKG_CONFIG_PATH} ${assimp_devel_PACKAGE_PATH}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH})
pkg_check_modules(assimp assimp_devel REQUIRED)
include_directories(${assimp_INCLUDE_DIRS})
link_directories(${assimp_LIBRARY_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# necessary for collada reader to create the temporary dae files due to limitations in the URDF geometry
check_function_exists(mkstemps HAVE_MKSTEMPS)
if( HAVE_MKSTEMPS )
  add_definitions("-DHAVE_MKSTEMPS")
endif()

rosbuild_add_boost_directories()

rosbuild_add_executable(collada_gazebo_gen src/collada_gazebo_gen.cpp)
rosbuild_link_boost(collada_gazebo_gen filesystem)
rosbuild_add_link_flags(collada_gazebo_gen -Wl,--no-as-needed )

rosbuild_add_executable(collada_to_graphviz src/collada_to_graphviz.cpp)
rosbuild_link_boost(collada_to_graphviz filesystem)
rosbuild_add_link_flags(collada_to_graphviz -Wl,--no-as-needed )

rosbuild_add_executable(collada_to_urdf src/collada_to_urdf.cpp)
rosbuild_link_boost(collada_to_urdf filesystem program_options)
rosbuild_add_link_flags(collada_to_urdf -Wl,--no-as-needed )
target_link_libraries(collada_to_urdf ${assimp_LIBRARIES})

