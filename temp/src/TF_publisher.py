#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
import math
import tf
import sys
from localization.msg import Marker
from tf import transformations as t

class TF_marker_publisher():
    def __init__(self):

        rospy.init_node("TF_marker_publisher",anonymous=True)
        self.marker = Marker()
        self.marker1 = Marker()
        self.br = tf.TransformBroadcaster()
        rospy.loginfo("TF publisher node is on")
        self.r = rospy.Rate(200)
        rospy.on_shutdown(self.shutdown)
        self.publish()
    def publish(self):
        while not rospy.is_shutdown():
            self.static_frames()
            self.r.sleep()
            self.dynamic_frames()
    def static_frames(self):
            self.br.sendTransform((1.8, 0, 0),(t.quaternion_from_euler(math.radians(90),0,math.radians(-90))),rospy.Time.now(),"Marker1","world")
            self.br.sendTransform((-1.8, 0, 0),(t.quaternion_from_euler(math.radians(90),0,math.radians(90))),rospy.Time.now(),"Marker2","world")
            self.br.sendTransform((0, 1.2, 0),(t.quaternion_from_euler(math.radians(90),0,0)),rospy.Time.now(),"Marker3","world")
            self.br.sendTransform((0, -1.2, 0),(t.quaternion_from_euler(math.radians(90),0,math.radians(180))),rospy.Time.now(),"Marker4","world")
    def dynamic_frames(self):
            self.marker=rospy.wait_for_message("aruco_markers", Marker)
            rospy.loginfo(self.marker.id)
            if self.marker.id==1:
                T_matrix_ = t.translation_matrix((self.marker.tvec.x,self.marker.tvec.y,self.marker.tvec.z))
                R_matrix_ = t.euler_matrix(self.marker.rvec.x,self.marker.rvec.y,self.marker.rvec.z)
                R_matrix_ = np.transpose(R_matrix_)
                Quaternion_ = t.quaternion_from_matrix(R_matrix_)
                Translation_ = (self.marker.tvec.x,self.marker.tvec.y,self.marker.tvec.z)
                self.br.sendTransform(Translation_, Quaternion_, rospy.Time.now(),"base_camera", "Marker1")

            if self.marker.id==2:
                T_matrix_ = t.translation_matrix((self.marker.tvec.x,self.marker.tvec.y,self.marker.tvec.z))
                R_matrix_ = t.euler_matrix(self.marker.rvec.x,self.marker.rvec.y,self.marker.rvec.z)
                R_matrix_ = np.transpose(R_matrix_)
                Quaternion_ = t.quaternion_from_matrix(R_matrix_)
                Translation_ = (self.marker.tvec.x,self.marker.tvec.y,self.marker.tvec.z)
                self.br.sendTransform(Translation_, Quaternion_, rospy.Time.now(),"base_camera", "Marker2")

            if self.marker.id==4:
                T_matrix_ = t.translation_matrix((self.marker.tvec.x,self.marker.tvec.y,self.marker.tvec.z))
                R_matrix_ = t.euler_matrix(self.marker.rvec.x,self.marker.rvec.y,self.marker.rvec.z)
                R_matrix_ = np.transpose(R_matrix_)
                Quaternion_ = t.quaternion_from_matrix(R_matrix_)
                Translation_ = (self.marker.tvec.x,self.marker.tvec.y,self.marker.tvec.z)
                self.br.sendTransform(Translation_, Quaternion_, rospy.Time.now(),"base_camera", "Marker3")

            if self.marker.id==3:
                T_matrix_ = t.translation_matrix((self.marker.tvec.x,self.marker.tvec.y,self.marker.tvec.z))
                R_matrix_ = t.euler_matrix(self.marker.rvec.x,self.marker.rvec.y,self.marker.rvec.z)
                R_matrix_ = np.transpose(R_matrix_)
                Quaternion_ = t.quaternion_from_matrix(R_matrix_)
                Translation_ = (self.marker.tvec.x,self.marker.tvec.y,self.marker.tvec.z)
                self.br.sendTransform(Translation_, Quaternion_, rospy.Time.now(),"base_camera", "Marker4")


    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("TF Marker Publisher Node is shutdown")
        rospy.sleep(1)

def main():
    try:
        TF_static_Publisher = TF_marker_publisher()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
