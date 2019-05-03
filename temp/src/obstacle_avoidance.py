#!/usr/bin/env python

from math import radians

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Obstacle():
    def __init__(self):
        rospy.init_node('obstacle')
        self.LIDAR_ERR = 0.05
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        rospy.on_shutdown(self.shutdown)

        self.obstacle()


    def get_scan(self):
        msg = rospy.wait_for_message("scan", LaserScan)
        self.scan_filter = []
        self.scan_filter_left = []
        self.scan_filter_right = []
        self.scan_filter_back = []
        for i in range(360):
            if i <= 15 or i > 335:
                if msg.ranges[i] >= self.LIDAR_ERR:
                    self.scan_filter.append(msg.ranges[i])
            if i >= 225 and i < 315:
                if msg.ranges[i] >= self.LIDAR_ERR:
                    self.scan_filter_right.append(msg.ranges[i])

            if i >= 35 and i < 125:
                if msg.ranges[i] >= self.LIDAR_ERR:
                    self.scan_filter_left.append(msg.ranges[i])

            if i >= 145 and i < 215:
                if msg.ranges[i] >= self.LIDAR_ERR:
                    self.scan_filter_back.append(msg.ranges[i])

    def move (self, forward, theta):

        self.twist.linear.x = forward
        self.twist.angular.z = radians(theta)
        self._cmd_pub.publish(self.twist)

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop - Robot is shutdown")
        self.move(0.0, 0.0)
        rospy.sleep(1)



    def obstacle(self):

        while not rospy.is_shutdown():
            self.get_scan()

            if min(self.scan_filter) < 0.3:
                self.move(0.0,0.0)
                rospy.loginfo('Stop!')
                rospy.sleep(0.5)
                index = 0
                counter = 0

                if min(self.scan_filter_left) > min(self.scan_filter_right):
                    self.move(0.0,45.0)
                    index = 1
                else:
                    self.move(0.0,-45.0)
                rospy.sleep(2)
                self.get_scan()

                rospy.loginfo('distance of the obstacle LEFT: %f', min(self.scan_filter_left))
                rospy.loginfo('distance of the obstacle RIGHT: %f', min(self.scan_filter_right))
                while min(self.scan_filter_left) <0.35 or min(self.scan_filter_right) < 0.35:

                    self.move(0.1,0.0)
                    self.get_scan()

                    if min(self.scan_filter) < 0.3 and index ==1:
                        self.move(0.0,45)
                        rospy.sleep(2)
                        rospy.loginfo('Obstacle in front Break1')
                        break
                    if min(self.scan_filter) < 0.3 and index ==0:
                        self.move(0.0,-45)
                        rospy.sleep(2)
                        rospy.loginfo('Obstacle in front Break1')
                        break

                    rospy.loginfo('distance of the obstacle LEFT: %f', min(self.scan_filter_left))
                    rospy.loginfo('distance of the obstacle RIGHT: %f', min(self.scan_filter_right))
                    rospy.sleep(0.1)
                    counter = counter+1

                rospy.loginfo('%d', counter)
                if index == 1:
                    self.move(0.0,-45)
                else:
                    self.move(0.0,45)
                rospy.sleep(2)
                self.move(0.0,0.0)

                rospy.loginfo('Up to this OK')

                rospy.loginfo(' ****distance of the obstacle LEFT: %f', min(self.scan_filter_left))
                rospy.loginfo(' ****distance of the obstacle RIGHT: %f', min(self.scan_filter_right))


                while min(self.scan_filter_left) <0.46 or min(self.scan_filter_right) < 0.46:

                    self.move(0.1,0.0)
                    rospy.sleep(0.15)
                    self.get_scan()
                    rospy.loginfo('distance of the obstacle LEFT: %f', min(self.scan_filter_left))
                    rospy.loginfo('distance of the obstacle RIGHT: %f', min(self.scan_filter_right))

                    if min(self.scan_filter) < 0.3 and index ==1:
                        rospy.loginfo('Obstacle in front Break2')
                        self.move(0.0,45)
                        rospy.sleep(2)
                        break
                    if min(self.scan_filter) < 0.3 and index ==0:
                        rospy.loginfo('Obstacle in front Break2')
                        self.move(0.0,-45)
                        rospy.sleep(2)
                        break

                if index == 1:
                    self.move(0.0,-45)

                else:
                    self.move(0.0,45)
                rospy.sleep(2)


                for i in range(counter):
                    self.move(0.1,0)
                    rospy.sleep(0.2)
                    if min(self.scan_filter) < 0.3 and index ==1:
                        self.move(0.0,45)
                        rospy.sleep(2)
                        rospy.loginfo('Obstacle in front Break3')
                        break
                    if min(self.scan_filter) < 0.3 and index ==0:
                        self.move(0.0,-45)
                        rospy.sleep(2)
                        rospy.loginfo('Obstacle in front Break2')
                        break
                self.move(0.0,0.0)
                rospy.sleep(1)
                if index == 1:
                    self.move(0.0,45)
                else :
                    self.move(0.0,-45)
                rospy.sleep(2)
                self.move(0.1,0)










                #if min(self.scan_filter_left) <= 0.5:
                #    self.twist.linear.x = 0.05
                #    self.twist.angular.z = 0.0
                #    self._cmd_pub.publish(self.twist)
                #else:
                #    self.twist.linear.x = 0.0
                #    self.twist.angular.z = 90
                #    rospy.loginfo('distance of the obstacle : %f', min(self.scan_filter))
                #    rospy.sleep(0.5)
                #    self.twist.linear.x = 0.05
                #    self._cmd_pub.publish(self.twist)


            else:
                self.move(0.05,0.0)
                rospy.loginfo('distance of the obstacle : %f', min(self.scan_filter))


def main():
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
