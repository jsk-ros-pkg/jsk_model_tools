if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_find_ros_package(colladadom)
if(NOT "${colladadom_PACKAGE_PATH}" STREQUAL "")
  set(COLLADA2EUS_LIBRARIES qhull yaml-cpp rospack)
else()
  find_package(PkgConfig)
  pkg_check_modules(COLLADA_DOM collada-dom-150)
  include_directories(${COLLADA_DOM_INCLUDE_DIRS})
  set(COLLADA2EUS_LIBRARIES qhull yaml-cpp rospack ${COLLADA_DOM_LIBRARIES})
endif()
rosbuild_add_executable(collada2eus src/collada2eus.cpp)
target_link_libraries(collada2eus ${COLLADA2EUS_LIBRARIES})

rosbuild_add_boost_directories()
rosbuild_find_ros_package(assimp_devel)
set(ENV{PKG_CONFIG_PATH} ${assimp_devel_PACKAGE_PATH}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH})
pkg_check_modules(assimp assimp_devel REQUIRED)
include_directories(${assimp_INCLUDE_DIRS})
link_directories(${assimp_LIBRARY_DIRS})
rosbuild_add_executable(collada2eus_urdfmodel src/collada2eus_urdfmodel.cpp)
target_link_libraries(collada2eus_urdfmodel ${COLLADA2EUS_LIBRARIES} ${assimp_LIBRARIES})
rosbuild_link_boost(collada2eus_urdfmodel filesystem program_options)

rosbuild_find_ros_package(euslisp)
macro(convert_to_collada)
  set(convert_exe "${CMAKE_CURRENT_SOURCE_DIR}/scripts/convert-irtmodel-to-collada.l")
  # search all irtmodels
  set(irtmodel_directory "${euslisp_PACKAGE_PATH}/jskeus/eus/models")
  set(collada_directory "${CMAKE_CURRENT_SOURCE_DIR}/collada")
  #
  message("convert scrpit ${convert_exe}")
  message("input irtmodel dir ${irtmodel_directory}")
  message("output collada dir ${collada_directory}")
  # get robot and object models
  file(GLOB irt_models "${irtmodel_directory}/*-robot.l" "${irtmodel_directory}/*-object.l")
  # remove scene object models
  file(GLOB irt_scenes RELATIVE ${irtmodel_directory} "${irtmodel_directory}/*-scene.l")
  foreach(_scene ${irt_scenes})
    string(REGEX REPLACE "\\-scene.l" "" _scene_name ${_scene})
    file(GLOB _scene_objects "${irtmodel_directory}/${_scene_name}-*-object.l")
    list(REMOVE_ITEM irt_models ${_scene_objects})
  endforeach(_scene ${irt_scenes})
  file(MAKE_DIRECTORY ${collada_directory})
  #
  foreach(_model ${irt_models})
    get_filename_component(_model_name ${_model} NAME_WE)
    set(_command_arg "\"\\(convert-irtmodel-to-collada \\\"${_model}\\\" \\\"${collada_directory}\\\" \\\"${_model_name}\\\"\\)\"")
    add_custom_command(OUTPUT "${collada_directory}/${_model_name}.dae"
      COMMAND "rosrun" "roseus" "roseus" "${convert_exe}" "\"${_command_arg}\""
      DEPENDS "${convert_exe}" "${_model}")
    list(APPEND _autogen "${collada_directory}/${_model_name}.dae")
  endforeach(_model)
  add_custom_target(CONVERT_EUS_TO_COLLADA DEPENDS ${_autogen})
  #add_dependencies(ROSBUILD_genmanifest_roseus_euscollada CONVERT_EUS_TO_COLLADA)
endmacro(convert_to_collada)

# if you want to convert all irteus models to collada models, please run "make -C `rospack find euscollada`/build CONVERT_EUS_TO_COLLADA"
convert_to_collada()

## pr2.l
rosbuild_find_ros_package(pr2_mechanism_model)
if(EXISTS "${pr2_mechanism_model_PACKAGE_PATH}")
  ADD_CUSTOM_COMMAND(
    OUTPUT ${CMAKE_SOURCE_DIR}/pr2.l
    COMMAND rosrun euscollada pr2.sh
    DEPENDS collada2eus)
  ADD_CUSTOM_TARGET(pr2model ALL DEPENDS ${CMAKE_SOURCE_DIR}/pr2.l)
endif()

## test
rosbuild_download_test_data(https://openrave.svn.sourceforge.net/svnroot/openrave/data/robots/mitsubishi-pa10.zae test/mitsubishi-pa10.zae 2e995a1356befd581028b9fd0d55b10f)
rosbuild_download_test_data(https://openrave.svn.sourceforge.net/svnroot/openrave/data/robots/unimation-pumaarm.zae test/unimation-pumaarm.zae 0d395a35da1f978163a75268b6595b98)

#rosbuild_download_test_data(https://openrave.svn.sourceforge.net/svnroot/openrave/data/robots/care-o-bot3.zae test/care-o-bot3.zae fc9d2e5c10a12b5988d7e43c36e8b639)
rosbuild_download_test_data(http://openrave.svn.sourceforge.net/viewvc/openrave/data/robots/care-o-bot3.zae?revision=3342&pathrev=3342 test/care-o-bot3.zae 36333fc202c2ca4b4169162a868d0f14)

rosbuild_download_test_data(https://openrave.svn.sourceforge.net/svnroot/openrave/data/robots/darpa-arm.zae test/darpa-arm.zae 3944246c010a4a3ab212506c0588895b)

rosbuild_download_test_data(https://openrave.svn.sourceforge.net/svnroot/openrave/data/robots/barrett-wam.zae test/barrett-wam.zae 7d59cad7186b3fb516c3dd75303c52f2)

rosbuild_add_pyunit(test/collada2eus_test.py)
rosbuild_check_for_display(disp)
rosbuild_find_ros_package(pr2_mechanism_model)
if(disp AND pr2_mechanism_model_FOUND)
  rosbuild_add_rostest(test/euscollada-test.launch)
endif()

# roslaunch documentation
# rosbuild_find_ros_package("jsk_tools")
# execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} python ${jsk_tools_PACKAGE_PATH}/bin/launchdoc-generator.py ${PROJECT_NAME} --output_dir=./build --nomakefile RESULT_VARIABLE _make_failed)
