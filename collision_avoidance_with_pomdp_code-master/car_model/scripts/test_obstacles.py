#!/usr/bin/python
import ConfigParser

import rospy
import numpy as np
import tf

from geometry_msgs.msg import Point32, Quaternion, Twist, Vector3, Point, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from costmap_converter.msg import ObstacleMsg, ObstacleArrayMsg
from Car_Node import VelocityGetter, ConfigSectionMap


class Position:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0


obst_array = ObstacleArrayMsg()

# Get parameters from the config
# Ini file parser
config = ConfigParser.ConfigParser()
config.read("/home/albert/ias_ros/src/car_model/config/config.ini")

MAX_VEL = float(ConfigSectionMap(config, "Pedestrian")['max_vel'])
PUBLISH_RATE = float(ConfigSectionMap(config, "Simulation")['publish_rate'])
ACCELERATION = float(ConfigSectionMap(config, "Pedestrian")['acceleration'])
START_POSITION = [float(ConfigSectionMap(config, "Pedestrian")['start_x']),
                  float(ConfigSectionMap(config, "Pedestrian")['start_y'])]
pedestrian_position = Position()


def create_obstacle():
    """
    Creates an invisible object, that holds positional information about the
    real object.
    :return:
    """
    obst_array.header.frame_id = "map"
    obst_array.header.stamp = rospy.Time.now()

    # Add obstacle to array
    obst = ObstacleMsg()
    obst.header.frame_id = "map"
    obst.header.stamp = rospy.Time.now()
    obst.id = 1
    obst.polygon.points.append(Point32(START_POSITION[0], START_POSITION[1], 0))
    obst.orientation = Quaternion(0, 0, 0, 1)
    obst.velocities.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

    obst_array.obstacles.append(obst)

    return obst_array


def create_interactive_marker(server):
    ##############
    i_marker = InteractiveMarker()
    i_marker.header.frame_id = "map"
    i_marker.header.stamp = rospy.Time.now()
    i_marker.name = "i_marker"
    i_marker.description = "Some Guy"
    i_marker.pose.position.x = START_POSITION[0]
    i_marker.pose.position.y = START_POSITION[1]
    i_marker.pose.position.z = 0.5

    # visual
    box_marker = Marker()
    box_marker.type = Marker.CYLINDER
    box_marker.id = 1
    box_marker.scale.x = 0.7
    box_marker.scale.y = 0.7
    box_marker.scale.z = 1
    box_marker.color.r = 0.5
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    # non-interactive control
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append(box_marker)
    i_marker.controls.append(box_control)

    move_control = InteractiveMarkerControl()
    move_control.name = "move_x"
    move_control.orientation.w = 1
    move_control.orientation.x = 0
    move_control.orientation.y = 1
    move_control.orientation.z = 0
    move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE

    i_marker.controls.append(move_control)

    server.insert(i_marker, feedback)
    server.applyChanges()


def feedback(fb):
    """
    Not used here. Can be used to do something, when position of the interactive marker
    changes.
    :param fb: Feedback (change) of the marker.
    """
    p = fb.pose.position
    # obst_array.obstacles[0].polygon.points[0].x = p.x
    # obst_array.obstacles[0].polygon.points[0].y = p.y


def cb_get_velocity(msg, vg):
    """ Callback method for getting the velocity as a Twist message.
        :param msg: the Twist message itself.
        :type msg:  Twist
        :param vg:  the velocity wrapper for storing the incoming velocity value so they
                    can be used in this node.
    """
    vg.vx = msg.linear.x
    vg.vy = msg.linear.y
    vg.vh = msg.angular.z


def cb_get_trigger_freeze(msg, obstacle_publisher):
    """
    Trigger for taking a snapshot of the current pedestrian position
    :type obstacle_publisher: rospy.Publisher
    :type msg: Bool
    """
    if msg.data == True:
        obst_array.header.stamp = rospy.Time.now()
        obst_array.obstacles[0].header.stamp = rospy.Time.now()
        obst_array.obstacles[0].polygon.points[0].x = pedestrian_position.x
        obst_array.obstacles[0].polygon.points[0].y = pedestrian_position.y
        obstacle_publisher.publish(obst_array)


# ========================================================================================
# Main Entry Point
# ========================================================================================
def main():
    rospy.init_node("interactive_obstacle", anonymous=True)
    rospy.loginfo("\033[1;32mStarting Interactive Marker Node !\033[0m\n")

    server = InteractiveMarkerServer("interactive_obstacle")

    # Publisher for publishing the obstacle array to the topic of the local planner
    pub_obstacle_array = rospy.Publisher("obstacles", ObstacleArrayMsg, queue_size=1)

    # Publisher for Odometry information
    odom_publisher = rospy.Publisher('pedestrian_position', Odometry, queue_size=10)

    # Subscriber for incoming velocity messages
    vg = VelocityGetter()
    rospy.Subscriber('cmd_vel_pedestrian', Twist, cb_get_velocity, callback_args=vg)

    # Subscriber: trigger for adding obstacle to obstacle vector and freeze position
    rospy.Subscriber("/freeze_obstacle_position", Bool, cb_get_trigger_freeze,
                     callback_args=pub_obstacle_array)

    # Create the obstacle array
    obst_array = create_obstacle()
    create_interactive_marker(server)

    # Save initial position temporarily
    x = START_POSITION[0]
    y = START_POSITION[1]
    th = 0
    pedestrian_position.x = x
    pedestrian_position.y = y

    last_time = rospy.Time.now()

    # Make Odometry information
    odom = Odometry()
    odom.header.frame_id = "map"

    rate = rospy.Rate(PUBLISH_RATE)
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        obst_array.header.stamp = rospy.Time.now()
        obst_array.obstacles[0].header.stamp = rospy.Time.now()

        # Update position of the pedestrian
        dt = (current_time - last_time).to_sec()
        dx = (vg.vx * np.cos(th) - vg.vy * np.sin(th)) * dt
        dy = (vg.vx * np.sin(th) + vg.vy * np.cos(th)) * dt
        dh = vg.vh * dt

        pedestrian_position.x += dx
        pedestrian_position.y += dy
        th += dh

        # Update time
        last_time = current_time

        # Update odometry information
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        odom.header.stamp = current_time
        odom.pose.pose = Pose(Point(pedestrian_position.x, pedestrian_position.y, 0),
                              Quaternion(*odom_quat))
        odom.twist.twist = Twist(Vector3(vg.vx, vg.vy, 0), Vector3(0, 0, vg.vh))
        odom_publisher.publish(odom)

        server.setPose("i_marker",
                       Pose(Point(pedestrian_position.x, pedestrian_position.y, 0.5),
                            Quaternion(*odom_quat)))
        server.applyChanges()

        rate.sleep()


if __name__ == '__main__':
    main()
