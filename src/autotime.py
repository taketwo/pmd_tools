#!/usr/bin/env python

PACKAGE = 'pmd_tools'
NODE = 'autotime'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from std_msgs.msg import UInt32
import dynamic_reconfigure.client
from pid import PID


class Fitness:
    def __init__(self):
        self.client = dynamic_reconfigure.client.Client("camera/driver",
                                                        timeout=30)
        self.saturated_sub = rospy.Subscriber('camera/saturation_filter/'
                                              'saturated_pixels', UInt32,
                                              self.saturated_cb)
        self.pid = PID(0.1, 0.0, 0.1)
        self.pid.setPoint(00)

    def set_integration_time(self, parameter):
        self.client.update_configuration({"integration_time": parameter})

    def saturated_cb(self, msg):
        pid = self.pid.update(msg.data)
        rospy.loginfo('Saturated: %f, Control: %f' % (msg.data, pid))
        time = self.client.get_configuration()['integration_time'] + pid
        if time < 30:
            time = 30
        if time > 2000:
            time = 2000
        self.set_integration_time(time)

if __name__ == '__main__':
    rospy.init_node(NODE)
    fitness = Fitness()
    rospy.loginfo('Started autotime node')
    rospy.spin()
