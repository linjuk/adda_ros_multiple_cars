#! /usr/bin/env python

import rospy
import tf
import time
import actionlib

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, TwistStamped, Pose, Point, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA, String

from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pomdp_car_msgs.srv import ActionObservation, ActionObservationRequest, \
    ActionObservationResponse


class TrajectoryInteractiveMarkers:

    def __init__(self):
        print('Listening of car2 position ...')
        self.count = 0
        self.pos_car2 = []
        self.listener = tf.TransformListener()

        self.goal_client2 = actionlib.SimpleActionClient('/car2/move_base', MoveBaseAction)
        self.goal_client2.wait_for_server()
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=100)
        rospy.Subscriber('/car2/cmd_vel', Twist, self.event_in_cb)

    def event_in_cb(self, msg):
        print('starting callback')
        self.waypoints = msg
        self.a = list()
        # print('blablabla', msg)
        # print(self.waypoints)
        self.a.append(msg.linear.x)
        self.a.append(msg.linear.y)
        self.a.append(msg.linear.z)

        # So you want to visualize velocity? Or what exactly do you want to do here?
        self.show_text_in_rviz()

    def show_text_in_rviz(self):
        # Inserted by me :D Maybe not as effective as your code, but let's see :D
        # -----------------------------------------------------------------
        marker = Marker()
        marker.header.frame_id = "car2/map"  # Shouldn't it be "/car2/map" or just "/map"?
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = self.count
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.position.x = self.a[0] / 10 ** 5
        marker.pose.position.y = self.a[1] / 10 ** 5
        marker.pose.position.z = self.a[2] / 10 ** 5
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 0.8
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.count += 1
        self.marker_publisher.publish(marker)
        print('Message published ...')

        # -----------------------------------------------------------------

        # print('starting to make setting to marker ')
        # self.marker = Marker()
        # self.marker.header.stamp = rospy.Time()
        # self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=5)

        # self.marker = Marker(
        #     type=Marker.SPHERE_LIST,
        #     id=0,
        #     lifetime=rospy.Duration(1000),
        #     pose=Pose(Point(self.a[0] / 10 ** 5, self.a[1] / 10 ** 5, self.a[2] / 10 ** 5),
        #               Quaternion(0, 0, 0, 1)),
        #     scale=Vector3(0.05, 0.05, 0.05),
        #     header=Header(frame_id='car2/map'),
        #     color=ColorRGBA(0.0, 2.0, 0.0, 0.8))
        # self.count += 1
        # self.marker.id = self.count
        # self.marker_publisher.publish(self.marker)
        # print('Message published ...')

    def tick(self):
        """
        Will be executed at every time step and processes TF Lookups
        """
        time_now = rospy.Time(0)
        (trans2, rot2) = self.listener.lookupTransform('/car2/base_link', '/map', time_now)
        self.pos_car2 = trans2
        print('car 2:', self.pos_car2)
        print('subscription sent')


if __name__ == '__main__':
    rospy.init_node('trajectory_viz')
    trajectory_interactive_markers = TrajectoryInteractiveMarkers()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        trajectory_interactive_markers.tick()
