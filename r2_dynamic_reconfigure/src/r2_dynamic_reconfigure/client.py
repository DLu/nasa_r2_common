import roslib; roslib.load_manifest('dynamic_reconfigure')
import rospy
import time
from dynamic_reconfigure.client import Client as DynamicReconfigureClient

class Client:
    def __init__(self, name, timeout=None):
        self.name = name
        self.client = DynamicReconfigureClient(name, timeout)

    def update(self, config):
        # send data out no greater than 10 times
        # revisit this and determine if this is actually needed
        c = 0
        new_config = {}
        while c < 10:
            # fuerte: update_configuration returns the configuration in a different format
            # get new_config and compare with sent out config
            updated_config = self.client.update_configuration(config)
            for key,val in config.iteritems():
                new_config[key] = updated_config[key]
            c += 1
            if new_config == config:
                break

        if c == 10:
            pre = 'failed to set '
        else:
            pre = 'successfully set '

        sleep_time = 0.05
        rospy.loginfo(pre + self.name + ' in ' + str(c) + ' attempt(s) and sleeping for ' + str(sleep_time) + 's')
        time.sleep(sleep_time)
