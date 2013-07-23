cmake_minimum_required(VERSION 2.8.3)
project(r2_dynamic_reconfigure)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/accelerationLimit.cfg
  cfg/armCartJoint.cfg
  cfg/auxCtrl.cfg
  cfg/cartDampingHigh.cfg
  cfg/cartIntegratorGain.cfg
  cfg/cartStiffnessHigh.cfg
  cfg/fingerImpedanceSwitch.cfg
  cfg/includeWaistJacobian.cfg
  cfg/jointDampingHigh.cfg
  cfg/jointDampingLow.cfg
  cfg/jointStiffnessHigh.cfg
  cfg/jointStiffnessLow.cfg
  cfg/kTendon.cfg
  cfg/kp.cfg
  cfg/lowPriorityJointRef.cfg
  cfg/taskNodes.cfg
  cfg/tendonBoost.cfg
  cfg/trajGenSwitch.cfg
  cfg/velocityLimit.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES r2_dynamic_reconfigure
  CATKIN_DEPENDS dynamic_reconfigure
#  DEPENDS system_lib
)

install(PROGRAMS
  nodes/fake_reconfigure_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/fake_reconfigure_server.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
