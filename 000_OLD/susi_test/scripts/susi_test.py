#! /usr/bin/env python


import rospy


import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Bool
import tf
import time

class SusiTest:

    def __init__(self):
        print "init"
        self.pos_car2 = []
        self.listener = tf.TransformListener()
        time.sleep(0.5)
        
        # setup publisher
        self._examplePublisher = rospy.Publisher('/susi_test/exampletopic', Bool, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.my_callback)

        self.belief 
        self.goals= []
        self.goals.append([0.1,5.0,4.0])
        self.goals.append([])

    def my_callback(self, event):


        #print 'Timer called at ' + str(event.current_real)

        time_now = rospy.Time(0)
        #(trans1, rot1) = self.listener.lookupTransform('/map', '/car1/base_link', time_now)
        (trans2, rot2) = self.listener.lookupTransform('/map', '/car2/base_link', time_now)
        #self.pos_car1 = trans1
        self.pos_car2 = trans2
        print self.pos_car2




   

if __name__ == '__main__':
    rospy.init_node("susi_test")
    bla = SusiTest()
    rospy.spin()
