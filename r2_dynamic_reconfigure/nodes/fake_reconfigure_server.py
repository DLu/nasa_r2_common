#! /usr/bin/env python
import roslib; roslib.load_manifest('r2_dynamic_reconfigure')
import rospy
import dynamic_reconfigure.server
import threading
import sys
import os
from dynamic_reconfigure.client import Client as DynamicReconfigureClient

# we don't know available configurations until at runtime, so import modules dynamically
import r2_dynamic_reconfigure.cfg
modules = []
for module in os.listdir(r2_dynamic_reconfigure.cfg.__path__[0]):
    if module[-3:] == '.py' and module != '__init__.py':
        modules.append(module[:-3])
_modules = __import__('r2_dynamic_reconfigure.cfg', globals(), locals(), modules, -1)

class FakeReconfigureServer():
    def __init__(self, param):
        rospy.init_node(param)
        self.param_config = {}
        for module in modules:
            self.param_config[module.split('Config')[0]] = getattr(_modules, module)
        self.server = dynamic_reconfigure.server.Server(self.param_config[param], self.reconfigure)
        while not rospy.is_shutdown():
            rospy.spin()

    def reconfigure(self, config, level):
        return config # Returns the updated configuration.

if __name__ == '__main__':
    try:
        # Get desired configuration from command-line argument
        param = sys.argv[1]
        FakeReconfigureServer(param)
    except rospy.ROSInterruptException:
        print 'broke'


