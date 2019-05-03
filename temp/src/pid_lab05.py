#!/usr/bin/env python

import rospy
import os
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf
import matplotlib.pyplot as plt
from localization.msg import Marker
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan


class  BOX_attach():
    def __init__(self):
        # initiliaze
        rospy.init_node('pid_turtle', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.marker = Marker()
        self.start_box_attach = Bool()
        self.lidar_front = 0
        self.twist = Twist()

        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.box_attach = rospy.Publisher('start_box_attach', Bool,queue_size=10)


        self.marker_sub = rospy.Subscriber("aruco_markers", Marker, self.callback1)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.callback2)

        #gain
        self.P = 1.2
        self.I = 0.4
        self.D = 0.4
        rospy.loginfo("Node PID Initialized")

        self.attach_to_box()

    def callback2(self, data):
        self.lidar_front = data.ranges[0]

    def callback1(self, data):
        if data.rvec.y <= 0.0:
            self.marker.rvec.y= -(np.pi+data.rvec.y)
        else:
            self.marker.rvec.y = -(data.rvec.y - np.pi)



    def attach_to_box(self):
        while not rospy.is_shutdown():
            self.start_box_attach = rospy.wait_for_message("start_box_attach", Bool)

            if self.start_box_attach.data == True:
                rospy.loginfo("True")
                self.imu_reading = rospy.wait_for_message("imu", Imu)
                self.euler_angle = tf.transformations.euler_from_quaternion([self.imu_reading.orientation.x,self.imu_reading.orientation.y,self.imu_reading.orientation.z,self.imu_reading.orientation.w])
                rospy.loginfo(self.euler_angle[2])
                self.marker = rospy.wait_for_message("aruco_markers", Marker)
                if self.marker.rvec.y <= 0.0:
                    self.marker.rvec.y= -(np.pi+self.marker.rvec.y)
                else:
                    self.marker.rvec.y = -(self.marker.rvec.y - np.pi)
                init_head_ = self.euler_angle[2]
                self.initial_heading = init_head_+self.marker.rvec.y # starting angle
                rospy.loginfo("After_wraping: %f",self.marker.rvec.y )

                attach_ = True
                integral_error_ = 0

                if attach_ == True:
                    self.twist.linear.x =0.029 # Move forward in a constant speed
                    self.cmd_vel.publish(self.twist)
                    while attach_:
                        rospy.loginfo ("initial_heading: %f", self.initial_heading)
                        self.imu_reading = rospy.wait_for_message("imu", Imu)
                        self.euler_angle = tf.transformations.euler_from_quaternion([self.imu_reading.orientation.x,self.imu_reading.orientation.y,self.imu_reading.orientation.z,self.imu_reading.orientation.w])
                        self.initial_heading = init_head_ + self.marker.rvec.y # starting angle

                        pid_error_ = self.initial_heading - self.euler_angle[2]
                        if pid_error_ < -np.pi:
                            pid_error_=2*np.pi-pid_error_
                        if pid_error_ > np.pi:
                            pid_error_=pid_error_-2*np.pi

                        u_ = self.P*pid_error_+self.I*integral_error_
                        rospy.loginfo("integral_error: %f", integral_error_)
                        rospy.loginfo ("p_error_: %f", pid_error_)

                        self.twist.angular.z = u_
                        self.cmd_vel.publish(self.twist)
                        integral_error_ = pid_error_

                        if self.lidar_front < 0.20 and self.lidar_front !=0.0 :
                            rospy.loginfo(" Something Else")
                            rospy.sleep(2.5)
                            self.cmd_vel.publish(Twist())
                            attach_ = False
                            self.start_box_attach.data = False
                            self.box_attach.publish(self.start_box_attach.data)

                rospy.loginfo("We exit")




#                self.twist.linear.x =0.05 # Move forward in a constant speed
#                self.cmd_vel.publish(self.twist)
#                integral_error_ = 0
#                self.imu_reading = rospy.wait_for_message("imu", Imu)
#                self.euler_angle = tf.transformations.euler_from_quaternion([self.imu_reading.orientation.x,self.imu_reading.orientation.y,self.imu_reading.orientation.z,self.imu_reading.orientation.w])
#                pid_error_ = self.initial_heading - self.euler_angle[2]
#                if pid_error_ < -np.pi:
#                    pid_error_=2*np.pi-pid_error_
#                    u_ = self.P*pid_error_+self.I*integral_error_
#                    rospy.loginfo("integral_error: %f", integral_error_)
#                    rospy.loginfo ("p_error_: %f", pid_error_)
#
#                    #Store for plotting
#                    self.error_P.append(pid_error_)
#                    self.theta.append(self.euler_angle[2])
#                    self.theta_t.append(self.initial_heading)
#                    self.u.append(u_)
#
#
#
#                    self.twist.angular.z = u_
#                    self.cmd_vel.publish(self.twist)
#                    integral_error_ = pid_error_



#    def PID(self):
#        self.twist.linear.x =0.3 # Move forward in a constant speed
#        self.cmd_vel.publish(self.twist)
#        integral_error_ = 0
#        while not rospy.is_shutdown():
#
#            self.imu_reading = rospy.wait_for_message("imu", Imu)
#            self.euler_angle = tf.transformations.euler_from_quaternion([self.imu_reading.orientation.x,self.imu_reading.orientation.y,self.imu_reading.orientation.z,self.imu_reading.orientation.w])
#            pid_error_ = self.initial_heading - self.euler_angle[2]
#            if pid_error_ < -np.pi:
#                pid_error_=2*np.pi-pid_error_
#        #    integral_error_ = integral_error_+pid_error_
#            u_ = self.P*pid_error_+self.I*integral_error_+self.D*self.imu_reading.angular_velocity.z
#
#            rospy.loginfo("derivative_error: %f", self.imu_reading.angular_velocity.z)
#            rospy.loginfo("integral_error: %f", integral_error_)
#            rospy.loginfo ("proportional_error: %f", pid_error_)
#
#            #Store for plotting
#            self.error_P.append(pid_error_)
#            self.theta.append(self.euler_angle[2])
#            self.theta_t.append(self.initial_heading)
#            self.u.append(u_)
#
#
#            self.twist.angular.z = u_
#            self.cmd_vel.publish(self.twist)
#            integral_error_ = pid_error_


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.loginfo("Stop - Robot is shutdown")
        rospy.sleep

def main():
    try:
        box_attach = BOX_attach()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
