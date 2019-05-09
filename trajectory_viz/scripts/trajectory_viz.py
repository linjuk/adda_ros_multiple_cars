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
from functions import comb_dataset, calculate_mean, calculate_covariance, calculate_std, belief_update_for_rviz

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

        self.dataset = comb_dataset(10000)
        self.all_means = calculate_mean(self.dataset)
        self.all_covariance = calculate_covariance(self.dataset)
        self.all_std = calculate_std(self.dataset)

        self.show_mean_right_in_rviz(opacity=0.3)
        self.show_mean_straight_in_rviz(opacity=0.3)
        self.show_mean_left_in_rviz(opacity=0.3)

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

    def show_mean_right_in_rviz(self, opacity):

        for i in range(0,10000,3):
            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "mean_right"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.scale.x = self.all_std[0][i][0] * 2
            marker.scale.y = self.all_std[0][i][1] * 2
            marker.scale.z = 0.1

            marker.pose.position.x = self.all_means[0][i][0]
            marker.pose.position.y = self.all_means[0][i][1]
            marker.pose.position.z = 0

            # print(self.all_means[0][self.count][0], self.all_means[0][self.count][1], 0)
            # marker.pose.position = Point(self.all_means[0][self.count][0], self.all_means[0][self.count][1], 0)
            marker.pose.orientation = Quaternion(0, 0, 0, 1)
            marker.color = ColorRGBA(0.0, 0.0, 0.5, opacity)

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

            self.marker_publisher_mean.publish(marker)

    def show_mean_straight_in_rviz(self, opacity):
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
        marker.color = ColorRGBA(0.0, 0.0, 0.5, opacity)
        self.count += 1
        self.marker_publisher.publish(marker)

    def show_mean_left_in_rviz(self, opacity):
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
        marker.color = ColorRGBA(0.0, 0.0, 0.5, opacity)
        self.count += 1
        self.marker_publisher.publish(marker)

############## DOES NOT WORK ##############
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

###########################################

    def show_belief_right_in_rviz(self):
        for i in range(3):
            marker = Marker()  # type: Marker()
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
        input_cordinates = [self.pos_car2[0], self.pos_car2[1]]
        mean_for_this_step = [self.all_means[0][self.belief_counter], self.all_means[1][self.belief_counter],
                              self.all_means[2][self.belief_counter]]
        covariance_for_this_step = [self.all_covariance[0][self.belief_counter],
                                    self.all_covariance[1][self.belief_counter],
                                    self.all_covariance[2][self.belief_counter]]

        print('belief counter', self.belief_counter)
        print('input cordinates', input_cordinates)
        print('mean', mean_for_this_step)
        print('covariance', covariance_for_this_step)

        calculated_belief = belief_update_for_rviz(input_cordinates, mean_for_this_step, covariance_for_this_step,
                                                   self.belief_array)
        self.belief_array.append(calculated_belief)
        print('calculated belief', calculated_belief)
        self.belief_counter += 1


        if calculated_belief[0] > calculated_belief[1] and calculated_belief[0] > calculated_belief[2]:
            self.show_mean_right_in_rviz(opacity=1)
            # self.show_mean_straight_in_rviz(opacity=0.1)
            # self.show_mean_left_in_rviz(opacity=0.1)
        elif calculated_belief[1] > calculated_belief[0] and calculated_belief[1] > calculated_belief[2]:
            # self.show_mean_right_in_rviz(opacity=0.1)
            self.show_mean_straight_in_rviz(opacity=1)
            # self.show_mean_left_in_rviz(opacity=0.1)
        elif calculated_belief[2] > calculated_belief[0] and calculated_belief[2] > calculated_belief[1]:
            # self.show_mean_right_in_rviz(opacity=0.1)
            # self.show_mean_straight_in_rviz(opacity=0.1)
            self.show_mean_left_in_rviz(opacity=0.1)

    def tick(self):
        """
        Will be executed at every time step and processes TF Lookups
        """
        try:
            (trans2, rot2) = self.listener.lookupTransform('/map', '/car2/base_link', rospy.Time(0))
            self.pos_car2 = trans2
            self.show_text_in_rviz()
            self.update_belief()
            self.show_belief_right_in_rviz()

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