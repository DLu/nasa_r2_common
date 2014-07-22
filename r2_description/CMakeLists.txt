cmake_minimum_required(VERSION 2.8.3)
project(r2_description)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES r2_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(DIRECTORY urdf robots meshes launch
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

