cmake_minimum_required(VERSION 2.8.3)
project(r2_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation message_runtime std_msgs visualization_msgs geometry_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Gains.msg
  JointControl.msg
  JointStatusArray.msg
  JointStatus.msg
  PDMCStatus.msg
  PoseCommandArray.msg
  PoseCommand.msg
  PoseCommandParams.msg
  PoseCommandStatus.msg
  PoseTwistStamped.msg
  TorsoStatus.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ParseTableScene.srv
  Power.srv
  ResetTableScene.srv
  Servo.srv
  SetJointMode.srv
  SetTipName.srv
  TakeSnapshot.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES r2_msgs
   CATKIN_DEPENDS std_msgs geometry_msgs visualization_msgs message_generation message_runtime
#  DEPENDS system_lib
)
