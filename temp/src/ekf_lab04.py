#!/usr/bin/env python

import rospy
import os
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField


class  Ekf():
    def __init__(self, initial_heading, pre_roll, pre_pitch, pre_yaw, pre_p): #Initialize with initial guess
        # initiliaze
        rospy.init_node('ekf', anonymous=True)
        rospy.loginfo("Node EKF Initialized")
        rospy.on_shutdown(self.shutdown)

        self.ekf_estimation = rospy.Publisher("ekf_estimation", MagneticField, queue_size=1) # Create a publisher to publish Roll Pitch and Yaw estimated values
        self.ekf_estimation_values = MagneticField()
        Time_Error_Constant_=rospy.wait_for_message("magnetic_field", MagneticField).header.stamp
        self.Time_Error_Constant = rospy.get_time()-(Time_Error_Constant_.secs+Time_Error_Constant_.nsecs*10e-10)
        #rospy.loginfo(self.Time_Error_Constant)
        self.magnetic_heading = initial_heading
        self.ts = 0.007
        self.Q =np.array([  [1.1339e-06,0.0,0.0]  ,  [0.0,3.1232e-006,0.0]  ,  [0.0,0.0,2.9009e-006]  ])*100
        self.R =np.array([  [0.01,0.0,0.0]  ,  [0.0,0.01,0.0]  ,  [0.0,0.0,0.01]  ])/10000

        #Magnetometer Calibration Matrix - Run that using calibrate_imu folder
        self.calibrate_mag_a = np.array( [[ 2676.92956613 , -226.10450081,  -339.64477868],
                                          [    0.         , 3029.16993399,   258.50438554],
                                          [    0.         ,    0.        ,  6718.20923656]])

        self.calbrate_mag_c = np.array( [[  3.72444015e-05], [  3.27637377e-04], [  4.69898215e-04]] )

        #Previous Step
        self.pre_roll = pre_roll
        self.pre_pitch = pre_pitch
        self.pre_yaw = pre_roll
        self.pre_P =pre_p #previous covariance matrix
        self.imu_reading = rospy.wait_for_message("imu", Imu)
        mag_field_reading_ = rospy.wait_for_message("magnetic_field", MagneticField)
        self.mag_field_reading = np.array([[mag_field_reading_.magnetic_field.x],
                                           [mag_field_reading_.magnetic_field.y],
                                           [mag_field_reading_.magnetic_field.z]])
        self.mag_field_reading = np.matmul(self.calibrate_mag_a,(self.mag_field_reading-self.calbrate_mag_c))



    def attitude_estimation(self):
        Roll_M_ = np.arctan2(self.imu_reading.linear_acceleration.y,self.imu_reading.linear_acceleration.x)
        Pitch_M_= np.arctan(-(self.imu_reading.linear_acceleration.x)/(self.imu_reading.linear_acceleration.y*np.sin(Roll_M_)+self.imu_reading.linear_acceleration.z*np.cos(Roll_M_)))
        y_temp1_=self.mag_field_reading[2][0]*np.sin(Roll_M_)-self.mag_field_reading[1][0]*np.cos(Roll_M_)
        y_temp2_= self.mag_field_reading[0][0]*np.cos(Pitch_M_)+self.mag_field_reading[1][0]*np.sin(Pitch_M_)*np.sin(Roll_M_)+self.mag_field_reading[2][0]*np.sin(Pitch_M_)*np.cos(Roll_M_)-self.magnetic_heading
        Yaw_M_ = np.arctan2(y_temp1_, y_temp2_)

        # Step #1: State Prediction (Dead Reconing)
        Roll_DR_=self.pre_roll+self.ts*(self.imu_reading.angular_velocity.x+self.imu_reading.angular_velocity.y*np.sin(self.pre_roll)*np.tan(self.pre_pitch)+self.imu_reading.angular_velocity.z*np.cos(self.pre_roll)*np.tan(self.pre_pitch))
        Pitch_DR_=self.pre_pitch+self.ts*(self.imu_reading.angular_velocity.y*np.cos(self.pre_roll)-self.imu_reading.angular_velocity.x*np.sin(self.pre_roll))
        Yaw_DR_=self.pre_yaw+self.ts*2*(self.imu_reading.angular_velocity.y*np.sin(self.pre_roll)+self.imu_reading.angular_velocity.z*np.cos(self.pre_roll))/np.cos(self.pre_pitch)

        # Step #2: Update Error Covariance minus
        # Calculate the Jacobian matrix
        A_k_= np.array([[1+self.ts*np.tan(self.pre_pitch)*(self.imu_reading.angular_velocity.y*np.cos(self.pre_roll)-self.imu_reading.angular_velocity.z*np.sin(self.pre_roll)) ,  self.ts*(1/np.power(np.cos(self.pre_pitch),2))*(self.imu_reading.angular_velocity.y*np.sin(self.pre_roll)+self.imu_reading.angular_velocity.z*np.cos(self.pre_roll))    ,    0],
        [-self.ts*(self.imu_reading.angular_velocity.y*np.sin(self.pre_roll)+self.imu_reading.angular_velocity.z*np.cos(self.pre_roll))       ,        1             ,                                        0],
        [self.ts*(self.imu_reading.angular_velocity.y*np.cos(self.pre_roll)-self.imu_reading.angular_velocity.z*np.sin(self.pre_roll))/np.cos(self.pre_pitch)  ,   self.ts*np.tan(self.pre_pitch)*(self.imu_reading.angular_velocity.y*np.sin(self.pre_roll)/np.cos(self.pre_pitch)+self.imu_reading.angular_velocity.z*np.cos(self.pre_roll)) , 1] ])

        Pm_k_=A_k_*self.pre_P*np.transpose(A_k_)+self.Q

        # Step #3: Update Feedback gain K
        K_k_=Pm_k_*np.linalg.inv(Pm_k_+self.R)

        # Step #4: Update States:

        Total_Acceleration_ =np.sqrt(np.power(self.imu_reading.linear_acceleration.x,2)+np.power(self.imu_reading.linear_acceleration.y,2)+np.power(self.imu_reading.linear_acceleration.z,2))      # Calculate the total acceleration
        Total_Mag_ =np.sqrt(np.power(self.mag_field_reading[0][0],2)+np.power(self.mag_field_reading[1][0],2)+np.power(self.mag_field_reading[2][0],2));                # Calculate the total magnetic measurement
        #rospy.loginfo("MagField: %f", Total_Mag_)
        #rospy.loginfo("Yaw_M: %f", Yaw_M_)
        #rospy.loginfo("YawDR: %f", Yaw_DR_)

            #rospy.loginfo(Total_Mag_)

        if Total_Acceleration_<1.05 and Total_Acceleration_>0.95:    # only update pitch and roll if the total acceleration is around 1g
            self.pre_roll=Roll_DR_+K_k[0,0]*(Roll_M_-Roll_DR_)
            self.pre_pitch=Pitch_DR_+K_k[1,1]*(Pitch_M_-Pitch_DR_)
            self.pre_P=(np.identity(3)-K_k_)*Pm_k_
        else: #% no update
            self.pre_roll=Roll_DR_
            self.pre_pitch=Pitch_DR_
            self.pre_P=Pm_k_

        if Total_Mag_<1.1 and Total_Mag_>0.9: # only update yaw if the total normalized magnetific field is around 1
            Yaw_Diff_=Yaw_M_-Yaw_DR_
            while Yaw_Diff_<-np.pi:                # wrap the angle
                Yaw_Diff_=Yaw_Diff_+2*np.pi
                rospy.loginfo("yes")
            while Yaw_Diff_>np.pi:
                Yaw_Diff_=Yaw_Diff_-2*np.pi;
                rospy.loginfo("yes2")
            rospy.loginfo("Yaw_Diff: %f",Yaw_Diff_)
            self.pre_yaw=Yaw_DR_+K_k_[2,2]*(Yaw_Diff_)
            self.pre_P=(np.identity(3)-K_k_)*Pm_k_
        else:
            self.pre_P=Pm_k_
            self.pre_yaw = Yaw_DR_

    def run_ekf(self):
        while(True):
            self.imu_reading = rospy.wait_for_message("imu", Imu)
            mag_field_reading_ = rospy.wait_for_message("magnetic_field", MagneticField)
            self.mag_field_reading = np.array([[mag_field_reading_.magnetic_field.x],
                                               [mag_field_reading_.magnetic_field.y],
                                               [mag_field_reading_.magnetic_field.z]])
            self.mag_field_reading = np.matmul(self.calibrate_mag_a,(self.mag_field_reading-self.calbrate_mag_c))

#            rospy.loginfo("mag_field_before_calbration: %f", mag_field_reading_.magnetic_field.z)
#            rospy.loginfo("mag_field_after__calbration: %f", self.mag_field_reading[2][0])
            ## MULTIPLE BY CALIBRATION
            ## UPDATE INITIAL READING
            self.attitude_estimation()
            self.ekf_estimation_values.header.stamp = rospy.get_rostime()
            self.ekf_estimation_values.magnetic_field.x = self.pre_roll
            self.ekf_estimation_values.magnetic_field.y = self.pre_pitch
            self.ekf_estimation_values.magnetic_field.z = self.pre_yaw
            self.ekf_estimation.publish(self.ekf_estimation_values)
            self.magnetic_heading = np.arctan2(-self.mag_field_reading[1][0],self.mag_field_reading[0][0])
            self.ts = rospy.get_time()-(self.imu_reading.header.stamp.secs+self.imu_reading.header.stamp.nsecs*10e-10)-self.Time_Error_Constant

    def shutdown(self):
        rospy.loginfo("Stop - Robot is shutdown")
        rospy.sleep

def main():
    try:
        ekf = Ekf(0.0000 , 0.00 , 0.0 , 0.0, np.identity(3))## PASS INITIAL READING
        ekf.run_ekf()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
