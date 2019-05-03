#! /usr/bin/python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Bool


class Turn_On_Electromagnet:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        rospy.init_node('turn_on_electromagnet', anonymous=True)
        rospy.loginfo("Node turn_on_electromagnet initiliazed")
        rospy.on_shutdown(self.shutdown) #what to do on shutdown
        self.magnetic_message = Bool()
        # GPIO pin number
        self.magnet_pin = 21
        # set magnet_pin as an output
        GPIO.setup(self.magnet_pin, GPIO.OUT)
        self.turn_on()

    def turn_on(self):
            while not rospy.is_shutdown():
                self.magnetic_message = rospy.wait_for_message("turn_on_magnet", Bool)
                if self.magnetic_message.data == True:
                    # turn GPIO pin on
                    rospy.loginfo("Turn On Magnet")
                    GPIO.output(self.magnet_pin, True)
                    rospy.sleep(0.5)
                if self.magnetic_message.data == False:
                    # turn GPIO pin off
                    rospy.loginfo("Turn off Magnet")
                    GPIO.output(self.magnet_pin, False)
                    rospy.sleep(0.5)
                rospy.sleep(0.05)

    def shutdown(self):
        rospy.loginfo("Stop - Node turn_on_electromagnet is shutdown")
        GPIO.cleanup()
        rospy.sleep

def main():
    try:
        turn_on_electromagnet = Turn_On_Electromagnet()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
