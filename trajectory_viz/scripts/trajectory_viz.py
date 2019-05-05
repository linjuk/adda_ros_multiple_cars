#! /usr/bin/env python

import rospy
from std_msgs.msg import ColorRGBA
import tf
import actionlib

from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, TwistStamped, Pose, Point, Vector3, Quaternion, PoseStamped


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pomdp_car_msgs.srv import ActionObservation, ActionObservationRequest, \
    ActionObservationResponse

# import os, sys
# sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
# from ..movement_prediction.functions import comb_dataset
import sys
sys.path.insert(0, '/home/linjuk/adda_ros/src/code_lina/movement_prediction/')
from functions import comb_dataset, calculate_mean, calculate_covariance, calculate_std

class TrajectoryInteractiveMarkers:

    def __init__(self):
        print('Listening of car2 position ...')
        self.count = 0
        self.pos_car2 = []
        self.listener = tf.TransformListener()

        self.goal_client2 = actionlib.SimpleActionClient('/car2/move_base', MoveBaseAction)
        self.goal_client2.wait_for_server()
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=100)
        # self.path_publisher = rospy.Publisher('/visualizaton_path', Path, queue_size=100)

        self.dataset = comb_dataset(10000)
        self.all_means = calculate_mean(self.dataset)
        self.all_covariance = calculate_covariance(self.dataset)
        self.all_std = calculate_std(self.dataset)

        # print("--- mean ---")
        # print(self.all_means) # [[all_right],[all_straight],[all_left]]
        # print("------------\n\n")
        #
        # print("--- covariance ---")
        # print(self.all_covariance)
        # print("------------\n\n")
        #
        # print("--- std ---")
        # print(self.all_std)
        # print("------------\n\n")

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
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.5)

        self.count += 1
        self.marker_publisher.publish(marker)
        # -----------------------------------------------------------------

    def show_mean_right_in_rviz(self):
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = self.count
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        for i in range(10000):
            point = Point()  # moved in to for loop
            point.x = self.all_means[0][i][0]
            point.y = self.all_means[0][i][1]
            point.z = 0
            marker.points.append(point)

        # print(self.all_means[0][self.count][0], self.all_means[0][self.count][1], 0)
        # marker.pose.position = Point(self.all_means[0][self.count][0], self.all_means[0][self.count][1], 0)
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(0.0, 0.0, 0.5, 0.5)

        # my_path = Path()
        # # my_path.frame_id = "map"
        # my_path.header.stamp = rospy.Time.now()
        # my_path.ns = "my_namespace"
        #
        # # posestamped list
        # pose_list = list()
        #
        # # make the poses into posestamped
        # for loc in range(500):
        #     loc = Pose()  # moved in to for loop
        #     loc.position.x = self.all_means[0][i][0]
        #     loc.position.y = self.all_means[0][i][1]
        #     loc.position.z = 0
        #
        #     pose = PoseStamped()
        #     pose.pose = loc
        #     pose_list.append(pose)
        #     my_path.poses.append(pose)
        #
        # my_path.poses.append(pose_list)

        self.count += 1
        self.marker_publisher.publish(marker)

    def show_mean_straight_in_rviz(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = self.count
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        for i in range(10000):
            point = Point()  # moved in to for loop
            point.x = self.all_means[1][i][0]
            point.y = self.all_means[1][i][1]
            point.z = 0
            marker.points.append(point)

        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(0.0, 0.0, 0.5, 0.5)
        self.count += 1
        self.marker_publisher.publish(marker)

    def show_mean_left_in_rviz(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = self.count
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        for i in range(10000):
            point = Point()  # moved in to for loop
            point.x = self.all_means[2][i][0]
            point.y = self.all_means[2][i][1]
            point.z = 0
            marker.points.append(point)

        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(0.0, 0.0, 0.5, 0.5)
        self.count += 1
        self.marker_publisher.publish(marker)


    def show_std_right_in_rviz(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = self.count
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        for i in range(10000):
            point = Point()  # moved in to for loop
            point.x = self.all_means[0][i][0]
            point.y = self.all_means[0][i][1]
            point.z = 2* self.all_std[0][i]
            marker.points.append(point)

        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(0.0, 0.0, 0.5, 0.5)
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
            self.show_mean_right_in_rviz()
            self.show_mean_straight_in_rviz()
            self.show_mean_left_in_rviz()
            self.show_std_right_in_rviz()
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
