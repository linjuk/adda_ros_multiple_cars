import os
import rospy
import rospkg
import math
import yaml
import roslib

import numpy as np
import time
import actionlib
import random
import math

from observation.msg import observation

def main():
    rospy.init_node('main')
    pub = rospy.Publisher('observation', observation, queue_size=10)
    rate = rospy.Rate(0.5) # 0.5hz - not sure if it is enough

    self.pos_car1 = []
    self.pos_car2 = []
    time_now = rospy.Time(0)
    (trans1, rot1) = self.listener.lookupTransform('/map', '/car1/base_link', time_now)
    (trans2, rot2) = self.listener.lookupTransform('/map', '/car2/base_link', time_now)
    self.pos_car1 = trans1
    self.pos_car2 = trans2

    while not rospy.is_shutdown():
        msg.pos_car1 = self.pos_car1 = []
        msg.pos_car2 = self.pos_car2 = []
        # msg.current_velocity = ??
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass