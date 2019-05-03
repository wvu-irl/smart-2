#!/usr/bin/env python

import rospy
import os
import pickle
import numpy as np
from sensor_msgs.msg import LaserScan

class laser_subscriber:
    def __init__(self):
        self.laser_scan_data =[]
        self.Subscriber()

    def Subscriber(self):
        while not rospy.is_shutdown():

            self.msg = rospy.wait_for_message("scan", LaserScan)
            rospy.loginfo('%s', len(self.msg.ranges))
            self.laser_scan_data.append(self.msg)


#            pickle.dump(self.laser_scan_data, open(self.config_dir + '/add_data', 'wb'))

if __name__ == '__main__':

    rospy.init_node('laser_subscriber')
    try:
        laser = laser_subscriber()
    except rospy.ROSInterruptException:
        pass
