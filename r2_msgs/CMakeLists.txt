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

catkin_package(
   CATKIN_DEPENDS 
    std_msgs 
    geometry_msgs
    visualization_msgs
    message_generation
    message_runtime
)
