#!/usr/bin/env python

import os
import yaml
import cv2
import cv2.aruco as aruco
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
import sys
from cv_bridge import CvBridge, CvBridgeError
from localization.msg import Marker

class ArucoROS():
    def __init__(self):
        self.config_dir = os.path.join(os.path.dirname(__file__))
        self.config_dir = self.config_dir.replace('src', 'src')
        rospy.init_node('aruco')
        self.bridge = CvBridge()
        self.data = Image()
        self.image = None
        self.image_sub = rospy.Subscriber("/smart2_01/usb_cam/image_raw",Image,self.callback)
        self.aruco_pub = rospy.Publisher('aruco_markers', Marker, queue_size=10)
        self.camera_info = yaml.load(open(self.config_dir+ "/calibration.yaml", 'r')) ### REMEMBER TO PUT THAT ON CAMERA INFO ADRESS
        self.marker = Marker()
        rospy.sleep(4)

        self.run()

    def run(self):
        while not rospy.is_shutdown():
            image_flipped_ = self.image.copy()
            cv2.flip(image_flipped_,-1,image_flipped_)
            (rows,cols,channels) = self.image.shape

            gray = cv2.cvtColor(image_flipped_, cv2.COLOR_BGR2GRAY, dstCn=0)
            print(gray.shape)

            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
            parameters =  aruco.DetectorParameters_create()
            print(parameters)
            #print(parameters)

            '''    detectMarkers(...)
            detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
            mgPoints]]]]) -> corners, ids, rejectedImgPoints
            '''
            #lists of ids and the corners beloning to each id
            ids = []
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners, 0.1, np.reshape(self.camera_info["camera_matrix"]["data"],(3,3)),     np.reshape(self.camera_info["distortion_coefficients"]["data"],5))
            #print(corners)
            if ids is not None:
                for id in range(len(ids)):
                    rospy.loginfo("Id: %d", ids[id])
                    #rospy.loginfo(corners[id])
                    print(rvec)
                    self.marker.header.stamp = rospy.get_rostime()
                    self.marker.header.frame_id = '/smart2_01/camera_link'
                    self.marker.id = ids[id]
                    self.marker.rvec.x = rvec[id][0][0];
                    self.marker.rvec.y = rvec[id][0][1];
                    self.marker.rvec.z = rvec[id][0][2];
                    self.marker.tvec.x = tvec[id][0][0];
                    self.marker.tvec.y = tvec[id][0][1];
                    self.marker.tvec.z = tvec[id][0][2];

                    self.aruco_pub.publish(self.marker)
                    #rospy.loginfo(np.shape(corners))

            gray = aruco.drawDetectedMarkers(gray, corners, ids)

            #print(rejectedImgPoints)
            # Display the resulting frame
            cv2.imshow('frame',gray)
            cv2.waitKey(3)

    def callback(self,data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.image = cv_image.copy()




def main():
    try:
        aruco_ros = ArucoROS()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
