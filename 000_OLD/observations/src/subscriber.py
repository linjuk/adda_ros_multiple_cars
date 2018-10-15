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
from std_msgs.msg import Float64, geometry_msgs

belief_update = Float64
result = Init16() # for testing

# object which going to be publisher
def callback(msg):
    #calculate belief update
    print('Receiving observation data')
    car1_x = msg.pos_car1[0]
    car1_y = msg.pos_car1[1]
    car2_x = msg.pos_car2[0]
    car2_y = msg.pos_car2[1]
    # vel_car1 = msg.current_velocity

    #testing if pub and sub working
    result.data = car1_x + car2_x



def main():
    rospy.init_node('subscriber')
    pub = rospy.Publisher("belief_update", Float64, queue_size=1) # publish result of belief
    sub = rospy.Subscriber("observation", Float64, callback) # subscribe observatio,
    r = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        pub.publish(belief_update)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

