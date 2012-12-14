#!/usr/bin/env python

PACKAGE = 'pmd_tools'
NODE = 'autotime3'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

import numpy as np
from pmd_tools.msg import Histogram
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
        self.hist_sub = rospy.Subscriber('/camera/distance_histogram/'
                                         'histogram', Histogram, self.hist_cb)

    def hist_cb(self, msg):
        for b, l in zip(msg.bins, msg.limits):
            if b > 10:
                t = np.polyval([6.58589420435121e+06, -8.45060362603123e+06,
                                3.80706136885209e+06, -7.64806310161187e+05,
                                9.25157270009183e+04, -4.52056517875628e+03,
                                1.04137050228125e+02], l)
                self.time.set(t)
                rospy.loginfo('Distance %.3f, time %i' % (l, int(t)))
                return


if __name__ == '__main__':
    rospy.init_node(NODE)
    saturation_pid = SaturationPID()
    rospy.loginfo('Started autotime3 node.')
    rospy.spin()
