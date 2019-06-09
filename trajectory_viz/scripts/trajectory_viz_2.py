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
from functions import comb_dataset, calculate_mean, calculate_covariance, calculate_std, belief_update_for_rviz, distance_formula, distance_formula_dtw

class TrajectoryInteractiveMarkers:

    def __init__(self):
        print('Listening of car2 position ...')
        self.count = 0
        self.belief_counter = 0
        self.pos_car2 = []
        self.belief_array = []
        self.listener = tf.TransformListener()

        self.goal_client2 = actionlib.SimpleActionClient('/car2/move_base', MoveBaseAction)
        self.goal_client2.wait_for_server()
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=100)
        self.marker_publisher_mean = rospy.Publisher('/marker_mean', Marker, queue_size=100)
        # self.path_publisher = rospy.Publisher('/visualizaton_path', Path, queue_size=100)

        self.dataset = comb_dataset(100)
        self.all_means = calculate_mean(self.dataset)
        self.all_covariance = calculate_covariance(self.dataset)
        self.all_std = calculate_std(self.dataset)

        self.show_mean_right_in_rviz(opacity=0.1)
        self.show_mean_straight_in_rviz(opacity=0.1)
        self.show_mean_left_in_rviz(opacity=0.1)

        rospy.Subscriber('/car2/cmd_vel', Twist, self.event_in_cb)

    def event_in_cb(self, msg):
        # print('starting callback')
        self.waypoints = msg
        self.a = list()
        self.a.append(msg.linear.x)
        self.a.append(msg.linear.y)
        self.a.append(msg.linear.z)

    def show_text_in_rviz(self):

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

    def show_mean_right_in_rviz(self, opacity):

        # for i in range(100):
        #     marker = Marker()
        #
        #     marker.header.frame_id = "map"
        #     marker.header.stamp = rospy.Time.now()
        #     marker.ns = "mean_right"
        #     marker.id = i
        #     marker.type = Marker.SPHERE
        #     marker.action = Marker.ADD
        #
        #     marker.scale.x = self.all_std[0][i][0] * 2
        #     marker.scale.y = self.all_std[0][i][1] * 2
        #     marker.scale.z = 0.1
        #
        #     marker.pose.position.x = self.all_means[0][i][0]
        #     marker.pose.position.y = self.all_means[0][i][1]
        #     marker.pose.position.z = 0
        #
        #     marker.pose.orientation = Quaternion(0, 0, 0, 1)
        #     marker.color = ColorRGBA(0.0, 0.0, 1.0, opacity)
        #
        #     self.marker_publisher_mean.publish(marker)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "mean_right"
        marker.id = self.count
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        for i in range(100):
            point = Point()  # moved in to for loop
            point.x = self.all_means[0][i][0]
            point.y = self.all_means[0][i][1]
            point.z = 0
            marker.points.append(point)

        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.scale = Vector3(self.all_std[0][i][0] * 2, self.all_std[0][i][1] * 2, 0.1)
        marker.color = ColorRGBA(0.0, 0.0, 1.0, opacity)
        self.count += 1
        marker.lifetime = rospy.Duration(0.25)
        self.marker_publisher_mean.publish(marker)

    def show_mean_straight_in_rviz(self, opacity):

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "mean_straight"
        marker.id = self.count
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        for i in range(100):
            point = Point()  # moved in to for loop
            point.x = self.all_means[1][i][0]
            point.y = self.all_means[1][i][1]
            point.z = 0
            marker.points.append(point)

        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.scale = Vector3(self.all_std[1][i][0] * 2, self.all_std[1][i][1] * 2, 0.1)
        marker.color = ColorRGBA(1.0, 0.0, 0.0, opacity)
        self.count += 1
        marker.lifetime = rospy.Duration(0.25)
        self.marker_publisher_mean.publish(marker)

    def show_mean_left_in_rviz(self, opacity):

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "mean_left"
        marker.id = self.count
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        for i in range(100):
            point = Point()  # moved in to for loop
            point.x = self.all_means[2][i][0]
            point.y = self.all_means[2][i][1]
            point.z = 0
            marker.points.append(point)

        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.scale = Vector3(self.all_std[2][i][0] * 2, self.all_std[2][i][1] * 2, 0.1)
        marker.color = ColorRGBA(0.0, 1.0, 0.0, opacity)
        self.count += 1
        marker.lifetime = rospy.Duration(0.25)
        self.marker_publisher_mean.publish(marker)

    def show_belief_in_rviz(self):
        for i in range(3):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "belief_text"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.text = "{:.4f}".format(self.belief_array[-1][i])
            x = self.all_means[i][-1][0]
            y = self.all_means[i][-1][1]
            marker.pose = Pose(Point(x, y, 1.0), Quaternion(0, 0, 0, 1))
            marker.scale = Vector3(1, 1, 1)
            marker.color = ColorRGBA(0.0, 0.0, 1.0, 1)
            self.marker_publisher.publish(marker)

    def update_belief(self):
        # required  input: input_cordinates, mean, covariance, belief_array
        t_right, t_left, t_straight = [0, 0, 0]
        all_distances_right = []
        all_distances_straight = []
        all_distances_left = []

        input_cordinates = [self.pos_car2[0], self.pos_car2[1]]

        for i in range(len(self.all_means[0])):
            distance_between_coordinates_right = distance_formula_dtw(input_cordinates, self.all_means[0][i])
            distance_between_coordinates_straight = distance_formula_dtw(input_cordinates, self.all_means[1][i])
            distance_between_coordinates_left = distance_formula_dtw(input_cordinates, self.all_means[2][i])

            all_distances_right.append(distance_between_coordinates_right)
            all_distances_straight.append(distance_between_coordinates_straight)
            all_distances_left.append(distance_between_coordinates_left)

        t_right = 0
        for i in range(len(all_distances_right)):
            if all_distances_right[i] < all_distances_right[t_right]:
                t_right = i


        t_straight = 0
        for i in range(len(all_distances_straight)):
            if all_distances_straight[i] < all_distances_straight[t_straight]:
                t_straight = i

        t_left = 0
        for i in range(len(all_distances_left)):
            if all_distances_left[i] < all_distances_left[t_left]:
                t_left = i


        print("t_right: ", t_right)
        print("t_straight: ", t_straight)
        print("t_left: ", t_left)


        mean_for_this_step = [self.all_means[0][t_right], self.all_means[1][t_straight],
                              self.all_means[2][t_left]]
        covariance_for_this_step = [self.all_covariance[0][t_right],
                                    self.all_covariance[1][t_straight],
                                    self.all_covariance[2][t_left]]

        print('input cordinates', input_cordinates)
        print('mean: ', mean_for_this_step)
        print('covariance" ', covariance_for_this_step)

        calculated_belief = belief_update_for_rviz(input_cordinates, mean_for_this_step, covariance_for_this_step,
                                                   self.belief_array)
        self.belief_array.append(calculated_belief)
        print('calculated belief: ', calculated_belief)
        #self.belief_counter += 1


        if calculated_belief[0] > calculated_belief[1] and calculated_belief[0] > calculated_belief[2]:
            self.show_mean_right_in_rviz(opacity=calculated_belief[0])
            self.show_mean_straight_in_rviz(opacity=calculated_belief[1])
            self.show_mean_left_in_rviz(opacity=calculated_belief[2])
        elif calculated_belief[1] > calculated_belief[0] and calculated_belief[1] > calculated_belief[2]:
            self.show_mean_right_in_rviz(opacity=calculated_belief[0])
            self.show_mean_straight_in_rviz(opacity=calculated_belief[1])
            self.show_mean_left_in_rviz(opacity=calculated_belief[2])
        elif calculated_belief[2] > calculated_belief[0] and calculated_belief[2] > calculated_belief[1]:
            self.show_mean_right_in_rviz(opacity=calculated_belief[0])
            self.show_mean_straight_in_rviz(opacity=calculated_belief[1])
            self.show_mean_left_in_rviz(opacity=calculated_belief[2])
        elif calculated_belief[0] == calculated_belief[1] and calculated_belief[0] == calculated_belief[2] and calculated_belief[1] == calculated_belief[2]:
            self.show_mean_right_in_rviz(opacity=calculated_belief[0])
            self.show_mean_straight_in_rviz(opacity=calculated_belief[1])
            self.show_mean_left_in_rviz(opacity=calculated_belief[2])


    def tick(self):
        """
        Will be executed at every time step and processes TF Lookups
        """
        try:
            (trans2, rot2) = self.listener.lookupTransform('/map', '/car2/base_link', rospy.Time(0))
            self.pos_car2 = trans2
            self.show_text_in_rviz()
            self.update_belief()
            self.show_belief_in_rviz()

        except (tf.LookupException, tf.ConnectivityException, tf.ConnectivityException):
            pass


if __name__ == '__main__':
    rospy.init_node('trajectory_viz')
    trajectory_interactive_markers = TrajectoryInteractiveMarkers()
    rospy.sleep(0.5)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
        trajectory_interactive_markers.tick()