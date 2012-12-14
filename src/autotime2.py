#!/usr/bin/env python

PACKAGE = 'pmd_tools'
NODE = 'autotime'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from std_msgs.msg import UInt32
import dynamic_reconfigure.client


class IntegrationTime:

    MIN = 30
    MAX = 2000

    def __init__(self):
        self.client = dynamic_reconfigure.client.Client("camera/driver",
                                                        timeout=1)

    def set(self, time):
        time = min(self.MAX, max(self.MIN, time))
        self.client.update_configuration({"integration_time": time})

    def get(self):
        return self.client.get_configuration()['integration_time']

    def add(self, time):
        current = self.get()
        self.set(current + time)


class SaturationPID:
    def __init__(self):
        self.time = IntegrationTime()
        self.saturated_sub = rospy.Subscriber('camera/depth_saturation_filter/'
                                              'saturated_pixels', UInt32,
                                              self.saturated_cb)

    def saturated_cb(self, msg):
        if not msg.data == 0:
            self.time.add(-1)
            rospy.loginfo('Saturated: %f, reducing time...' % (msg.data))


if __name__ == '__main__':
    rospy.init_node(NODE)
    saturation_pid = SaturationPID()
    rospy.loginfo('Started autotime2 node.')
    rospy.spin()
