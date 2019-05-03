#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
import math
import tf
import sys
import cv2
from localization.msg import Marker
from tf import transformations as t

class TF_marker_publisher():
    def __init__(self):

        rospy.init_node("TF_marker_publisher",anonymous=True)
        self.marker = Marker()
        self.br = tf.TransformBroadcaster()
        rospy.loginfo("TF publisher node is on")
        self.r = rospy.Rate(200)
        rospy.on_shutdown(self.shutdown)
        self.publish()
    def publish(self):
        while not rospy.is_shutdown():
            self.r.sleep()
            self.box_frames()

    def box_frames(self):
            self.marker=rospy.wait_for_message("aruco_markers", Marker) # Possible to subscribe to other topic where the box markers are being published
            rospy.loginfo(self.marker.id)

   
            if self.marker.id==0:
                self.br.sendTransform(self.translation(self.marker), self.quaternion(self.marker), rospy.Time.now(),"Marker_01", "smart2_01/camera_link")


                ## Follow the same pattern for other boxes


    def translation(self, marker_):
        return (marker_.tvec.x,marker_.tvec.y,marker_.tvec.z)

    def quaternion(self, marker_):
        rodrigues = cv2.Rodrigues(np.array([marker_.rvec.x,marker_.rvec.y,marker_.rvec.z]))
        rodrigues = rodrigues[0]#np.transpose(rodrigues[0])
        a = np.zeros([4,4])
        a[3,3]=1
        a[0:3,0:3] = rodrigues

        
        print('Rodrigues:')


        
        R_matrix_ = t.euler_matrix(marker_.rvec.x,marker_.rvec.y,marker_.rvec.z,'rzxz')
        R_matrix_ = np.transpose(R_matrix_)
        print(R_matrix_)
        #print(t.quaternion_from_euler(marker_.rvec.x,marker_.rvec.y,marker_.rvec.z))
        #print(t.quaternion_from_matrix(R_matrix_))
        return t.quaternion_from_matrix(a)

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
