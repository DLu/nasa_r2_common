#! /usr/bin/env python
import roslib;
roslib.load_manifest('r2_dynamic_reconfigure')
import rospy
import dynamic_reconfigure.server
import threading
import sys
from dynamic_reconfigure.client import Client as DynamicReconfigureClient

# TODO: Import config dynamically, or import all configs, or something
from r2_dynamic_reconfigure.cfg import accelerationLimitConfig
from r2_dynamic_reconfigure.cfg import auxCtrlConfig
from r2_dynamic_reconfigure.cfg import cartDampingHighConfig
from r2_dynamic_reconfigure.cfg import jointDampingHighConfig
from r2_dynamic_reconfigure.cfg import jointDampingLowConfig
from r2_dynamic_reconfigure.cfg import cartIntegratorGainConfig
from r2_dynamic_reconfigure.cfg import armCartJointConfig
from r2_dynamic_reconfigure.cfg import cartStiffnessHighConfig
from r2_dynamic_reconfigure.cfg import jointStiffnessHighConfig
from r2_dynamic_reconfigure.cfg import jointStiffnessLowConfig
from r2_dynamic_reconfigure.cfg import kpConfig
from r2_dynamic_reconfigure.cfg import kTendonConfig
from r2_dynamic_reconfigure.cfg import lowPriorityJointRefConfig
from r2_dynamic_reconfigure.cfg import taskNodesConfig
from r2_dynamic_reconfigure.cfg import trajGenSwitchConfig
from r2_dynamic_reconfigure.cfg import velocityLimitConfig
from r2_dynamic_reconfigure.cfg import includeWaistJacobianConfig
from r2_dynamic_reconfigure.cfg import tendonBoostConfig
from r2_dynamic_reconfigure.cfg import fingerImpedanceSwitchConfig

params = {}
params['accelerationLimit'] = accelerationLimitConfig
params['auxCtrl'] = auxCtrlConfig
params['cartDampingHigh'] = cartDampingHighConfig
params['jointDampingHigh'] = jointDampingHighConfig
params['jointDampingLow'] = jointDampingLowConfig
params['cartIntegratorGain'] = cartIntegratorGainConfig
params['armCartJoint'] = armCartJointConfig
params['cartStiffnessHigh'] = cartStiffnessHighConfig
params['jointStiffnessHigh'] = jointStiffnessHighConfig
params['jointStiffnessLow'] = jointStiffnessLowConfig
params['kp'] = kpConfig
params['kTendon'] = kTendonConfig
params['lowPriorityJointRef'] = lowPriorityJointRefConfig
params['taskNodes'] = taskNodesConfig
params['trajGenSwitch'] = trajGenSwitchConfig
params['velocityLimit'] = velocityLimitConfig
params['includeWaistJacobian'] = includeWaistJacobianConfig
params['tendonBoost'] = tendonBoostConfig
params['fingerImpedanceSwitch'] = fingerImpedanceSwitchConfig

class FakeReconfigureServer():
    def __init__(self, param):
        rospy.init_node(param)
        self.server = dynamic_reconfigure.server.Server(params[param], self.reconfigure)
        while not rospy.is_shutdown():
            rospy.spin()

    def print_config(self, config):
        for k, v in config.iteritems():
            print k, ":", v

    def config_callback(self, config): 
        print "Got callback, configuration is: "
        print_config(config)

    def reconfigure(self, config, level):
        return config # Returns the updated configuration.

if __name__ == '__main__':
    try:
        # Get desired configuration from command-line argument
        param = sys.argv[1]
        FakeReconfigureServer(param)
    except rospy.ROSInterruptException:
        print 'broke'


