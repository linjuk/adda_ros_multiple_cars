#! /usr/bin/env python

import rospy
from std_msgs.msg import ColorRGBA
import tf
import actionlib

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, TwistStamped, Pose, Point, Vector3, Quaternion


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pomdp_car_msgs.srv import ActionObservation, ActionObservationRequest, \
    ActionObservationResponse

# import os, sys
# sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
# from ..movement_prediction.functions import comb_dataset
import sys
sys.path.insert(0, '/home/linjuk/adda_ros/src/code_lina/movement_prediction/')
from functions import comb_dataset, calculate_mean, calculate_covariance

class TrajectoryInteractiveMarkers:

    def __init__(self):
        print('Listening of car2 position ...')
        self.count = 0
        self.pos_car2 = []
        self.listener = tf.TransformListener()

        self.goal_client2 = actionlib.SimpleActionClient('/car2/move_base', MoveBaseAction)
        self.goal_client2.wait_for_server()
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=100)

        self.dataset = comb_dataset(100)
        self.all_means = calculate_mean(self.dataset)
        self.all_covariance = calculate_covariance(self.dataset)

        print(self.all_means)

        rospy.Subscriber('/car2/cmd_vel', Twist, self.event_in_cb)

    def event_in_cb(self, msg):
        # print('starting callback')
        self.waypoints = msg
        self.a = list()
        # print('blablabla', msg)
        # print(self.waypoints)
        self.a.append(msg.linear.x)
        self.a.append(msg.linear.y)
        self.a.append(msg.linear.z)

    def show_text_in_rviz(self):
        # -----------------------------------------------------------------
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = self.count
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Vector3(self.pos_car2[0], self.pos_car2[1], self.pos_car2[2])
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.5)

        self.count += 1
        self.marker_publisher.publish(marker)
        # -----------------------------------------------------------------

    def show_mean_in_rviz(self):
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = self.count
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Vector3(self.all_means[0], self.all_means[1], self.all_means[2])
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.5)

        self.count += 1
        self.marker_publisher.publish(marker)



    def tick(self):
        """
        Will be executed at every time step and processes TF Lookups
        """
        try:
            (trans2, rot2) = self.listener.lookupTransform('/map', '/car2/base_link', rospy.Time(0))
            self.pos_car2 = trans2
            self.show_text_in_rviz()
            # self.show_mean_in_rviz()
        except (tf.LookupException, tf.ConnectivityException, tf.ConnectivityException):
            pass


if __name__ == '__main__':
    rospy.init_node('trajectory_viz')
    trajectory_interactive_markers = TrajectoryInteractiveMarkers()
    rospy.sleep(0.5)

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        rate.sleep()
        trajectory_interactive_markers.tick()
