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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_find_ros_package(euslisp)
# for using convex decomposition in openrave/3rdparty
rosbuild_find_ros_package(openrave)
include_directories(/usr/include /usr/X11R6/include ${euslisp_PACKAGE_PATH}/jskeus/eus/include ${openrave_PACKAGE_PATH}/openrave_git/3rdparty/convexdecomposition)
link_directories ("${openrave_PACKAGE_PATH}/build/3rdparty/convexdecomposition")

rosbuild_add_library(eus_assimp src/eus_assimp.cpp)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
if(EXISTS ${openrave_PACKAGE_PATH}/build/3rdparty/convexdecomposition/libconvexdecomposition.a)
  target_link_libraries(eus_assimp convexdecomposition)
  add_definitions(-DCOMPILE_CONVEX_DECOMPOSITION=1)
endif()

set(COMPILE_FLAGS "-O2 -Wno-write-strings -Wno-comment")
add_definitions(-DLINUX -DLinux -D_REENTRANT -DVERSION='\"${8.26}\"' -DTHREADED -DPTHREAD -DX11R6_1)
add_definitions(-DSVNVERSION='\"r${SVNVERSION}\"')
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR
   ${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* )
 add_definitions(-Dx86_64)
else()
 add_definitions(-Di486)
endif()

rosbuild_add_compile_flags(eus_assimp ${COMPILE_FLAGS})
set_target_properties(eus_assimp PROPERTIES PREFIX "" SUFFIX ".so")
