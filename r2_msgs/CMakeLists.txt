cmake_minimum_required(VERSION 2.8.3)
project(r2_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  visualization_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  Gains.msg
  JointControl.msg
  JointStatus.msg
  JointStatusArray.msg
  PDMCStatus.msg
  PoseCommand.msg
  PoseCommandArray.msg
  PoseCommandParams.msg
  PoseCommandStatus.msg
  PoseTwistStamped.msg
  TorsoStatus.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ParseTableScene.srv
  Power.srv
  ResetTableScene.srv
  Servo.srv
  SetJointMode.srv
  SetTipName.srv
  TakeSnapshot.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  std_msgs
  visualization_msgs
  message_generation
)
