#!/usr/bin/python

import rospy
import tty
import sys
import termios

from std_msgs.msg import String
# from config import *

# For Keyboard Input purposes...
orig_settings = termios.tcgetattr(sys.stdin)
tty.setraw(sys.stdin)


class Controller():
    """ This node handles the user input from the terminal and publishes the keystrokes. """

    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.publisher = rospy.Publisher('controller', String, queue_size=100)
        self.rate = rospy.Rate(20)

        self.x = ''
        rospy.loginfo('---------- Controller Node Started ----------\n')

    def get_key(self):
        """ Returns the key the user currently entered. """
        self.x = sys.stdin.read(1)[0]
        return self.x

    def sleep(self):
        """ Sleep function of Rate class. """
        self.rate.sleep()

    def publish(self):
        self.publisher.publish(self.x)


# ---------------- MAIN ENTRY POINT ------------------
def main():
    controller = Controller()
    while not rospy.is_shutdown():
        x = controller.get_key()
        if x == 'q':
            break
        controller.publish()
        controller.sleep()

    rospy.loginfo("\nKilling Controller Node !\n")
    # Restore original terminal settings ...
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)


if __name__ == '__main__':
    main()