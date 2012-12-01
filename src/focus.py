#!/usr/bin/env python

PACKAGE = 'pmd_tools'
NODE = 'focus'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from pmd_tools.msg import PlaneFitResult
import dynamic_reconfigure.client
import numpy as np
from scipy.optimize import minimize_scalar


class Fitness:
    def __init__(self):
        self.client = dynamic_reconfigure.client.Client("camera/driver",
                                                        timeout=30)
        self.accumulating = False

    def target_function(self, parameter):
        self.client.update_configuration({"integration_time": parameter})
        self.area = list()
        self.std = list()
        self.accumulating = True
        while self.accumulating:
            pass
        area = np.array(self.area)
        area_mean = np.mean(area)
        area_std = np.std(area)
        std = np.array(self.std)
        std_median = np.median(std)
        rospy.loginfo('Area std %.5f mean %.5f Std median %.5f' % (area_std,
                                                                   area_mean,
                                                                   std_median))
        rospy.loginfo('Score %.8f' % (1.0 * std_median))
        return 1.0 * std_median# / area_mean

    def fit_cb(self, msg):
        if self.accumulating:
            self.area.append(msg.area)
            self.std.append(msg.std)
            if len(self.area) == 30:
                self.accumulating = False


if __name__ == '__main__':
    rospy.init_node(NODE)
    fitness = Fitness()
    fit_sub = rospy.Subscriber('fit', PlaneFitResult, fitness.fit_cb)
    rospy.loginfo('Started focus node')
    res = minimize_scalar(fitness.target_function, bounds=(12, 2000),
                          method='Bounded', options={'disp': True})
    print res
