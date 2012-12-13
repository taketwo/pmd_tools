#!/usr/bin/env python

PACKAGE = 'pmd_tools'
NODE = 'autotime'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from std_msgs.msg import UInt32
import dynamic_reconfigure.client
from pid import PID


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
        self.saturated_sub = rospy.Subscriber('camera/saturation_filter/'
                                              'saturated_pixels', UInt32,
                                              self.saturated_cb)
        self.pid = PID(0.1, 0.0, 0.1)
        self.pid.setPoint(00)

    def saturated_cb(self, msg):
        pid = self.pid.update(msg.data)
        rospy.loginfo('Saturated pixels: %f, Control: %f' % (msg.data, pid))
        self.time.add(pid)

if __name__ == '__main__':
    rospy.init_node(NODE)
    saturation_pid = SaturationPID()
    rospy.loginfo('Started autotime node')
    rospy.spin()
