cmake_minimum_required(VERSION 2.8.3)
project(r2_control)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  nodes/r2_fullbody_ready_pose.py nodes/r2_ready_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

