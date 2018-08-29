#!/usr/bin/python
import rospy
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from config import *


class Motor:
    """ This class subscribes to Controller_Node to retrieve the keystrokes from the user and
        interpret them as accelerations and decelerations. It publishes the new acceleration. """

    def __init__(self):
        rospy.init_node("acceleration_sender")
        self.publisher = rospy.Publisher("cmd_vel_pedestrian", Twist, queue_size=100)

        # Subscriptions
        rospy.Subscriber('controller', String, self.get_key)
        self.rate = rospy.Rate(PUBLISH_RATE)
        self.velocity = 0.0

        # Initialize acceleration vector to 0
        self.acceleration = Twist()
        self.acceleration.linear.x = 0.0
        self.acceleration.linear.y = 0.0

    def get_key(self, key):
        """ Retrieves the keystrokes from the controller node and stores the appropriate acceleration command. """

        inp = ''
        try:
            inp = str(key).split()[1]
        except IOError:
            # To prevent errors, we just don't accelerate
            self.acceleration.linear.x = 0.0
            self.acceleration.linear.y = 0.0
            self.publisher.publish(self.acceleration)

        # We're accelerating in x direction
        if inp == '\"w\"':
            self.acceleration.linear.x = 10

        # We're accelerating in negative x direction
        elif inp == '\"s\"':
            self.acceleration.linear.x = -10

        # We're accelerating in y direction
        elif inp == '\"a\"':
            self.acceleration.linear.y = 10

        # We're accelerating in negative y direction
        elif inp == '\"d\"':
            self.acceleration.linear.y = -10

        # We are not increasing or decreasing the acceleration. Therefore, hold the velocity.
        else:
            self.acceleration.linear.x = 0.0
            self.acceleration.linear.y = 0.0

    def publish(self):
        self.publisher.publish(self.acceleration)
        self.acceleration.linear.x = 0.0
        self.acceleration.linear.y = 0.0


if __name__ == '__main__':
    monitor = Motor()
    while not rospy.is_shutdown():
        monitor.publish()
        monitor.rate.sleep()
