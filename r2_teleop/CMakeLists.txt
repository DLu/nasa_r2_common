cmake_minimum_required(VERSION 2.8.3)
project(r2_teleop)

find_package(catkin REQUIRED COMPONENTS geometry_msgs interactive_markers r2_msgs sensor_msgs tf)
find_package(orocos_kdl REQUIRED)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES r2_teleop
  CATKIN_DEPENDS geometry_msgs interactive_markers r2_msgs sensor_msgs tf
  DEPENDS orocos_kdl
)

install(PROGRAMS
  nodes/r2_interactive_control.py
  nodes/r2_interactive_sensing.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
