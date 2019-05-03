#!/usr/bin/env python

import rospy
import numpy as np
import tf
import sys
from tf import transformations as t
from tf import TransformListener
from geometrys_msgs.msg import Pose

class Get_transform():
    def __init__(self):
        self.pose = Pose()
        rospy.init_node("get_transform",anonymous=True)
        self.tf = TransformListener()
        self.pub = rospy.publisher("/box_pose",Pose)
        rospy.loginfo("TF Listener node is on")
        self.r = rospy.Rate(200)
        rospy.on_shutdown(self.shutdown)
        self.get_transform()
    def get_transform(self):
        while not rospy.is_shutdown():
            self.r.sleep()
            if self.tf.frameExists("map") and self.tf.frameExists("Box1R"):
                transformation_ = self.tf.getLatestCommonTime("map", "Box1R")
                position_, quaternion_ = self.tf.lookupTransform("map", "Box1R", transformation_)
                self.pose.position = position_
                self.pose.orientation = quaternion_
                self.pub.publish(self.pose)


    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("TF Marker Publisher Node is shutdown")
        rospy.sleep(1)

def main():
    try:
        get_transform = Get_transform()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
