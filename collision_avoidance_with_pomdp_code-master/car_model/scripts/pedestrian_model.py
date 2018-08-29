#!/usr/bin/python
import ConfigParser

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist, Vector3, TransformStamped, Pose, Point, Quaternion, TwistWithCovariance, Point32
from nav_msgs.msg import Odometry
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from std_msgs.msg import Bool

from Car_Node import ConfigSectionMap
from Car_Node import VelocityGetter

# ----------------------------------------------------------------------------------------
# This file will handle the transition model of the pedestrian in the environment.
# It acts as a controller node, which accepts velocity commands as Twist messages.
# ----------------------------------------------------------------------------------------

# Ini file parser
config = ConfigParser.ConfigParser()
config.read("/home/albert/ias_ros/src/car_model/config/config.ini")


class PedPosition:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0


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


obst_array = ObstacleArrayMsg()
inst_ped_position = PedPosition()


def create_obstacle():
    obst_array.header.frame_id = "map"
    obst_array.header.stamp = rospy.Time.now()

    # One obstacle in this array
    obst = ObstacleMsg()
    obst.header.frame_id = "map"
    obst.header.stamp = rospy.Time.now()
    obst.id = 1

    obst_x = inst_ped_position.x
    obst_y = inst_ped_position.y

    obst.polygon.points.append(Point32(obst_x, obst_y, 0))
    obst.orientation = Quaternion(0, 0, 0, 1)
    obst.velocities.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

    obst_array.obstacles.append(obst)

    return obst_array


def cb_get_trigger_freeze(msg, obstacle_publisher):
    """
    Trigger for taking a snapshot of the current pedestrian position
    :type msg: Bool
    """
    print "Got something!"
    if msg.data:
        obst_array.obstacles[0].polygon.points[0].x = inst_ped_position.x
        obst_array.obstacles[0].polygon.points[0].y = inst_ped_position.y
        # print obst_array.obstacles[0].polygon.points[0].x
        # print obst_array.obstacles[0].polygon.points[0].y
        obstacle_publisher.publish(obst_array)


# Get parameters from the config
MAX_VEL = float(ConfigSectionMap(config, "Pedestrian")['max_vel'])
PUBLISH_RATE = float(ConfigSectionMap(config, "Simulation")['publish_rate'])
ACCELERATION = float(ConfigSectionMap(config, "Pedestrian")['acceleration'])


def main():
    rospy.init_node('pedestrian_model_node', anonymous=True)
    rospy.loginfo("\033[1;32mStarting Pedestrian Node !\033[0m\n")

    # Publisher for Odometry information
    odom_publisher = rospy.Publisher('pedestrian_position', Odometry, queue_size=10)

    # Publisher for TebLocalPlanner for dynamic obstacle detection
    obst_publisher = rospy.Publisher("obstacles", ObstacleArrayMsg, queue_size=1)

    # Subscriber for incoming velocity commands
    vg = VelocityGetter()
    rospy.Subscriber('cmd_vel_pedestrian', Twist, cb_get_velocity, callback_args=vg)

    # Subscriber: trigger for adding obstacle to obstacle vector and freeze position
    rospy.Subscriber("freeze_obstacle_position", Bool, cb_get_trigger_freeze, callback_args=obst_publisher)

    # TF Broadcaster
    odom_broadcaster = tf.TransformBroadcaster()

    # Get the starting position for the car from the configuration file.
    inst_ped_position.x = float(ConfigSectionMap(config, "Pedestrian")['start_x'])
    inst_ped_position.y = float(ConfigSectionMap(config, "Pedestrian")['start_y'])
    inst_ped_position.th = 0

    last_time = rospy.Time.now()
    rate = rospy.Rate(PUBLISH_RATE)

    # Initialize transformation and odometry
    odom_trans = TransformStamped()
    odom_trans.header.frame_id = 'map'
    odom_trans.child_frame_id = 'ped_base_link'

    odom = Odometry()
    odom.header.frame_id = 'map'
    odom.child_frame_id = 'ped_base_link'

    obst_array = create_obstacle()

    # Main loop
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # Calculate changed velocity by integrating over time step dt
        dt = (current_time - last_time).to_sec()
        dx = (vg.vx * np.cos(inst_ped_position.th) - vg.vy * np.sin(inst_ped_position.th)) * dt
        dy = (vg.vx * np.sin(inst_ped_position.th) + vg.vy * np.cos(inst_ped_position.th)) * dt
        dh = vg.vh * dt

        # Update the new position
        inst_ped_position.x += dx
        inst_ped_position.y += dy
        inst_ped_position.th += dh

        # Quaternion for the angular position
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, inst_ped_position.th)

        # TF Broadcaster Setup
        odom_trans.header.stamp = current_time

        # Gather all information about the current position
        odom_trans.transform.translation = (inst_ped_position.x, inst_ped_position.y, 0.0)
        odom_trans.transform.rotation = odom_quat

        # Broadcast the TF
        odom_broadcaster.sendTransform(odom_trans.transform.translation,
                                       odom_trans.transform.rotation,
                                       odom_trans.header.stamp, odom_trans.child_frame_id,
                                       odom_trans.header.frame_id)

        # Odometry Publisher Setup
        odom.header.stamp = current_time

        # Gather positional and velocity information
        odom.pose.pose = Pose(Point(inst_ped_position.x, inst_ped_position.y, 0.0), Quaternion(*odom_quat))
        odom.twist.twist = Twist(Vector3(vg.vx, vg.vy, 0.0), Vector3(0, 0, vg.vh))
        odom_publisher.publish(odom)

        last_time = current_time

        obst_array.obstacles[0].polygon.points[0].x = inst_ped_position.x
        obst_array.obstacles[0].polygon.points[0].y = inst_ped_position.y
        # print obst_array.obstacles[0].polygon.points[0].x
        # print obst_array.obstacles[0].polygon.points[0].y

        obst_publisher.publish(obst_array)
        # print obst_array.obstacles[0].polygon.points[0].x
        # print obst_array.obstacles[0].polygon.points[0].y
        rate.sleep()


if __name__ == '__main__':
    main()
