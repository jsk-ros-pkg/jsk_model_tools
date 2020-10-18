cmake_minimum_required(VERSION 3.0.2)
project(eusurdfwrl)

find_package(catkin REQUIRED)

catkin_package()

include(cmake/eusurdfwrl.cmake)
convert_urdf_to_wrl()
convert_gazebo_world_to_environment_yaml()
