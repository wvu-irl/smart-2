#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import radians



class DrawASquare():
    def __init__(self):
        # initiliaze
        rospy.init_node('stop', anonymous=True)

        # What to do you ctrl + c    
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
	
    try:
        DrawASquare()
    except:
        rospy.loginfo("node terminated.")

