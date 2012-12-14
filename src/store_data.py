#!/usr/bin/env python

PACKAGE = 'pmd_tools'
NODE = 'store_data'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

import dynamic_reconfigure.client
from pmd_tools.msg import Histogram
from std_srvs.srv import Empty


class IntegrationTime:
    def __init__(self):
        self.client = dynamic_reconfigure.client.Client("camera/driver",
                                                        timeout=1)

    def get(self):
        return self.client.get_configuration()['integration_time']


class StoreDataNode:
    def __init__(self):
        self.hist_sub = rospy.Subscriber('/camera/distance_histogram/'
                                         'histogram', Histogram, self.hist_cb)
        self.time = IntegrationTime()
        self.save_srv = rospy.Service('~save', Empty, self.save_cb)
        self.data = open('calib.dat', 'w')

    def hist_cb(self, msg):
        for b, l in zip(msg.bins, msg.limits):
            if b > 10:
                self.current = (l, self.time.get())
                rospy.loginfo('Distance %.3f, time %i' % self.current)
                return

    def save_cb(self, req):
        self.data.write('%.3f %i\n' % self.current)
        return []

    def shutdown(self):
        self.data.close()

if __name__ == '__main__':
    rospy.init_node(NODE)
    sdn = StoreDataNode()
    rospy.on_shutdown(lambda: sdn.shutdown())
    rospy.spin()
