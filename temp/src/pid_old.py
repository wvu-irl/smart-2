#!/usr/bin/env python

import rospy
import os
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf
import matplotlib.pyplot as plt

class  PID_turtlebot():
    def __init__(self):
        # initiliaze
        rospy.init_node('pid_turtle', anonymous=True)
        rospy.loginfo("Node PID Initialized")
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.imu_reading = rospy.wait_for_message("imu", Imu)
        self.twist = Twist()
        rospy.loginfo(self.imu_reading.orientation.x) ##Z AXIS IS UP, THEREFORE TURN LEFT POSITIVE, RIGHT NEGATIVE
        self.euler_angle = tf.transformations.euler_from_quaternion([self.imu_reading.orientation.x,self.imu_reading.orientation.y,self.imu_reading.orientation.z,self.imu_reading.orientation.w])
        rospy.loginfo(self.euler_angle[2])
        self.initial_heading = self.euler_angle[2] # starting angle
        #gain
        self.P = 8
        self.I = 1.2

        self.D = 0.4



        ## STORE FOR PLOTTING
        self.error_P = []
        self.theta = []
        self.theta_t = []
        self.u = []




    def Proportional(self):
        self.twist.linear.x =0.1 # Move forward in a constant speed
        self.cmd_vel.publish(self.twist)
        while not rospy.is_shutdown():

            self.imu_reading = rospy.wait_for_message("imu", Imu)
            self.euler_angle = tf.transformations.euler_from_quaternion([self.imu_reading.orientation.x,self.imu_reading.orientation.y,self.imu_reading.orientation.z,self.imu_reading.orientation.w])
            pid_error_ = self.initial_heading - self.euler_angle[2]
            rospy.loginfo ("pid_error_: %f", pid_error_)

            if pid_error_ < -np.pi:
                pid_error_=2*np.pi-pid_error_
            u_ = self.P*pid_error_
            rospy.loginfo ("p_error_: %f", pid_error_)


            #store for plotting
            self.error_P.append(pid_error_)
            self.theta.append(self.euler_angle[2])
            self.theta_t.append(self.initial_heading)
            self.u.append(u_)

            self.twist.angular.z = u_
            self.cmd_vel.publish(self.twist)

    def PI(self):
        self.twist.linear.x =0.3 # Move forward in a constant speed
        self.cmd_vel.publish(self.twist)
        integral_error_ = 0
        while not rospy.is_shutdown():
            self.imu_reading = rospy.wait_for_message("imu", Imu)
            self.euler_angle = tf.transformations.euler_from_quaternion([self.imu_reading.orientation.x,self.imu_reading.orientation.y,self.imu_reading.orientation.z,self.imu_reading.orientation.w])
            pid_error_ = self.initial_heading - self.euler_angle[2]
            if pid_error_ < -np.pi:
                pid_error_=2*np.pi-pid_error_
            u_ = self.P*pid_error_+self.I*integral_error_
            rospy.loginfo("integral_error: %f", integral_error_)
            rospy.loginfo ("p_error_: %f", pid_error_)

            #Store for plotting
            self.error_P.append(pid_error_)
            self.theta.append(self.euler_angle[2])
            self.theta_t.append(self.initial_heading)
            self.u.append(u_)



            self.twist.angular.z = u_
            self.cmd_vel.publish(self.twist)
            integral_error_ = pid_error_

    def PD(self):
        self.twist.linear.x =0.3 # Move forward in a constant speed
        self.cmd_vel.publish(self.twist)
        while not rospy.is_shutdown():
            self.imu_reading = rospy.wait_for_message("imu", Imu)
            self.euler_angle = tf.transformations.euler_from_quaternion([self.imu_reading.orientation.x,self.imu_reading.orientation.y,self.imu_reading.orientation.z,self.imu_reading.orientation.w])
            pid_error_ = self.initial_heading - self.euler_angle[2]
            if pid_error_ < -np.pi:
                pid_error_=2*np.pi-pid_error_
            u_ = self.P*pid_error_+self.D*self.imu_reading.angular_velocity.z
            rospy.loginfo("derivative_error: %f", self.imu_reading.angular_velocity.z)
            rospy.loginfo ("p_error_: %f", pid_error_)

            #Store for plotting
            self.error_P.append(pid_error_)
            self.theta.append(self.euler_angle[2])
            self.theta_t.append(self.initial_heading)
            self.u.append(u_)


            self.twist.angular.z = u_
            self.cmd_vel.publish(self.twist)

    def PID(self):
        self.twist.linear.x =0.3 # Move forward in a constant speed
        self.cmd_vel.publish(self.twist)
        integral_error_ = 0
        while not rospy.is_shutdown():

            self.imu_reading = rospy.wait_for_message("imu", Imu)
            self.euler_angle = tf.transformations.euler_from_quaternion([self.imu_reading.orientation.x,self.imu_reading.orientation.y,self.imu_reading.orientation.z,self.imu_reading.orientation.w])
            pid_error_ = self.initial_heading - self.euler_angle[2]
            if pid_error_ < -np.pi:
                pid_error_=2*np.pi-pid_error_
        #    integral_error_ = integral_error_+pid_error_
            u_ = self.P*pid_error_+self.I*integral_error_+self.D*self.imu_reading.angular_velocity.z

            rospy.loginfo("derivative_error: %f", self.imu_reading.angular_velocity.z)
            rospy.loginfo("integral_error: %f", integral_error_)
            rospy.loginfo ("proportional_error: %f", pid_error_)

            #Store for plotting
            self.error_P.append(pid_error_)
            self.theta.append(self.euler_angle[2])
            self.theta_t.append(self.initial_heading)
            self.u.append(u_)


            self.twist.angular.z = u_
            self.cmd_vel.publish(self.twist)
            integral_error_ = pid_error_

    def plot_pid(self):
        x_ = np.array (range(len(self.error_P)))
        plt.step(x_, self.error_P, label="error")
        plt.step(x_, self.theta, label = "theta")
        plt.step(x_, self.theta_t, label="Initial_reading")
        plt.step(x_, self.u, label='u')
        plt.legend()
        plt.xlabel('steps')
        plt.ylabel('Radians')
        plt.show()


    def shutdown(self):
        self.cmd_vel.publish(Twist())

        self.plot_pid()

        rospy.loginfo("Stop - Robot is shutdown")
        rospy.sleep

def main():
    try:
        pid = PID_turtlebot()

        pid.PID()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
