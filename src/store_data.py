#!/usr/bin/env python

PACKAGE = 'pmd_tools'
NODE = 'store_data'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from pmd_tools.msg import Histogram
from std_srvs.srv import Empty
from integration_time import IntegrationTime


class StoreDataNode:
    def __init__(self):
        self.hist_sub = rospy.Subscriber('/camera/distance_histogram/'
                                         'histogram', Histogram, self.hist_cb)
        self.time = IntegrationTime()
        self.find_threshold = rospy.ServiceProxy('/desaturator/find_threshold',
                                                 Empty)
        self.save_srv = rospy.Service('~save', Empty, self.save_cb)
        self.data = open('calibration.dat', 'a')
        #self.timer = rospy.Timer(rospy.Duration(5), self.timer_cb)

    def hist_cb(self, msg):
        for b, l in zip(msg.bins, msg.limits):
            if b > 10:
                self.current = (l, self.time.get())
                return

    #def timer_cb(self, event):
    def save_cb(self, req):
        rospy.loginfo('Finding threshold...')
        self.find_threshold()
        rospy.loginfo('Done, saving %.3f -- %i' % (self.current))
        self.data.write('%.3f %i\n' % self.current)
        return []

    def shutdown(self):
        self.data.close()

if __name__ == '__main__':
    rospy.init_node(NODE)
    sdn = StoreDataNode()
    rospy.on_shutdown(lambda: sdn.shutdown())
    rospy.spin()
