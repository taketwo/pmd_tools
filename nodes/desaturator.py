#!/usr/bin/env python

PACKAGE = 'pmd_tools'
NODE = 'desaturator'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from collections import deque
from std_msgs.msg import UInt32
from std_srvs.srv import Empty
from integration_time import IntegrationTime


class Desaturator:
    def __init__(self):
        self.time = IntegrationTime()
        self.desaturate_srv = rospy.Service('~desaturate', Empty,
                                            self.desaturate_cb)
        self.find_threshold_srv = rospy.Service('~find_threshold', Empty,
                                                self.find_threshold_cb)
        self.saturated_sub = rospy.Subscriber('camera/depth_saturation_filter/'
                                              'saturated_pixels', UInt32,
                                              self.saturated_cb)
        self.saturated = 0
        self.history = deque([0] * 10)

    def saturated_cb(self, msg):
        self.saturated = msg.data
        self.history.append(msg.data)
        self.history.popleft()

    def desaturate_cb(self, req):
        r = rospy.Rate(30)
        while self.saturated > 0 or sum(self.history) > 0:
            rospy.loginfo('Saturated: %i' % (self.saturated))
            self.time.add(-1)
            r.sleep()
        return []

    def find_threshold_cb(self, req):
        r = rospy.Rate(5)
        min = self.time.MIN
        max = self.time.MAX
        while True:
            rospy.loginfo('Saturated: %i, Search range: [%i - %i]' %
                          (self.saturated, min, max))
            m = (min + max) / 2
            self.time.set(m)
            r.sleep()
            if self.saturated == 0:
                min = m + 1
            else:
                max = m - 1
            if max < min:
                break
        self.desaturate_cb(None)
        return []

if __name__ == '__main__':
    rospy.init_node(NODE)
    d = Desaturator()
    rospy.spin()
