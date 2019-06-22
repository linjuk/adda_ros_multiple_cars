#! /usr/bin/env python
import rospy
import csv
import os
import sys
from geometry_msgs.msg import PoseStamped

class Listener:
    def __init__(self):
        print('Listening... ')
        self.positions_array = []

        # setup subscriber
        rospy.Subscriber("/adma_pose_stamped", PoseStamped, self.callback)

    def __del__(self):
        with open('/home/linjuk/adda_ros/src/code_lina/Real data/0_RealData_old_left.csv', 'w+') as csvFile:
            writer = csv.writer(csvFile)
            writer.writerows(self.positions_array)
        csvFile.close()

        print('Bye!')

    def callback(self, msg):
        x_pos = msg.pose.position.x
        y_pos = msg.pose.position.y
        x_transformed = 0
        y_transformed = 0

        if len(self.positions_array) != 0:
            x_transformed = x_pos - self.positions_array[len(self.positions_array)-1][0] + self.positions_array[len(self.positions_array)-1][2]
            y_transformed = y_pos - self.positions_array[len(self.positions_array)-1][1] + self.positions_array[len(self.positions_array)-1][3]

        self.positions_array.append([x_pos, y_pos, x_transformed, y_transformed])
        print("Position: ", x_pos, y_pos)
        print("Transformed position: ", x_transformed, y_transformed)

if __name__ == '__main__':
    rospy.init_node("listener")
    test = Listener()
    rospy.spin()
