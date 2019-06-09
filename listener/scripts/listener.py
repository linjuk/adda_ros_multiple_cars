#! /usr/bin/env python
import rospy
import csv
import os
import sys
#import actionlib
from geometry_msgs.msg import PoseStamped

class Listener:
    def __init__(self):
        print('Listening... ')
        self.positions_array = []

        # setup subscriber
        rospy.Subscriber("/adma_pose_stamped", PoseStamped, self.callback)

    def __del__(self):
        with open('/home/linjuk/adda_ros/src/code_lina/Real data/left.csv', 'w+') as csvFile:
            writer = csv.writer(csvFile)
            writer.writerows(self.positions_array)

        csvFile.close()
        print('Bye!')

    def callback(self, msg):
        x_pos = msg.pose.position.x
        y_pos = msg.pose.position.y
        self.positions_array.append([x_pos, y_pos])
        print("Position: ", x_pos, y_pos)

if __name__ == '__main__':
    rospy.init_node("listener")
    test = Listener()
    rospy.spin()
