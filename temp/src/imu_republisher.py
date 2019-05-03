#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from math import radians

import os

import numpy as np
from nav_msgs.msg import Odometry

class ImuRep():
    def __init__(self):
        # initiliaze
        rospy.init_node('IMU', anonymous=True)

        # What to do you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
        self.imu_pub = rospy.Publisher('smart2_01/new_imu', Imu, queue_size=10)
     
	# 10 HZ
        r = rospy.Rate(10);

	# create two different Twist() variables.  One for moving forward.  One for turning 45 degrees.

        # let's go forward at 0.2 m/s
        imu_data = Imu()
 
        while not rospy.is_shutdown():
            imu_data =rospy.wait_for_message("smart2_01/imu/data", Imu)
            imu_data.angular_velocity_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            imu_data.linear_acceleration_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            imu_data.header.frame_id = 'smart2_01/base_footprint'
            imu_data.header.stamp = rospy.get_rostime()
            self.imu_pub.publish(imu_data)
        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop Imu Republisher ")
        rospy.sleep(1)
 
if __name__ == '__main__':
	
    try:
        ImuRep()
    except:
        rospy.loginfo("node terminated.")

