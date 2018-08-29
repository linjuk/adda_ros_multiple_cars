#!/usr/bin/python
import rospy
import ConfigParser
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist, Vector3


class IniParser:
    def __init__(self):
        self.config = ConfigParser.ConfigParser()
        self.config.read("/home/albert/ias_ros/src/car_model/config/config.ini")

    def get_config(self):
        return self.config

    def get(self, section):
        dict1 = {}
        options = self.config.options(section)
        for option in options:
            try:
                dict1[option] = self.config.get(section, option)
                if dict1[option] == -1:
                    print("skip: %s" % option)
            except IOError:
                print("exception on %s!" % option)
                dict1[option] = None
        return dict1


class Goal:
    def __init__(self):
        self.goals = {'zero_position': [0.0, 0.0]}

    def add_goal(self, name, position):
        if not (name in self.goals.keys()):
            self.goals[name] = position

    def get_goal_by_name(self, name):
        """
        Returns the position of the given goal name.
        :type name: basestring
        :param name: Name of the goal to get.
        """
        assert isinstance(name, basestring)
        if name in self.goals.keys():
            return self.goals[name]
        else:
            print("No goal position with name %s!" % name)


class Path:
    def __init__(self, starting_point):
        self.path = [starting_point]

    def add_point_to_path(self, point):
        """
        Adds a point to the path.

        :type point: Point
        """
        self.path.append(point)

    def remove_point(self, point):
        if point in self.path:
            self.path.remove(point)

    def print_path(self):
        for p in self.path:
            print p.x, p.y, ", "


class Pedestrian:
    def __init__(self):
        self.ini_parser = IniParser()

        # Visual Publisher
        self.publisher = rospy.Publisher("pedestrian_marker", Marker, queue_size=100)
        self.publisher_text = rospy.Publisher("pedestrian_text", Marker, queue_size=100)

        # Position Publisher
        self.pub_position = rospy.Publisher("pedestrian_position", Vector3, queue_size=100)

        # Visual Marker
        self.marker = Marker()
        self.text_marker = Marker()

        # Velocity as a Twist message
        self.vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        # Start position of the pedestrian from the config file
        self.start_position = Vector3(float(self.ini_parser.get("Pedestrian")['start_x']),
                                      float(self.ini_parser.get("Pedestrian")['start_y']),
                                      0.5)

        self.current_position = self.start_position

        self.goal = Vector3()

        self.init_marker()

    def publish(self):
        """
        Publishes all visual Markers and the position of the pedestrian at once.
        """
        self.marker.pose.position.x = self.current_position.x
        self.marker.pose.position.y = self.current_position.y

        self.text_marker.pose.position.x = self.current_position.x
        self.text_marker.pose.position.y = self.current_position.y

        self.publisher.publish(self.marker)
        self.publisher_text.publish(self.text_marker)
        self.pub_position.publish(self.current_position)

    def init_marker(self):
        """
        Initializes the Marker and its properties.
        """
        self.marker.header.frame_id = "map"
        self.marker.ns = 'ped'
        self.marker.header.stamp = rospy.Time.now()
        self.marker.type = Marker.CYLINDER
        self.marker.pose.position.x = self.start_position.x
        self.marker.pose.position.y = self.start_position.y
        self.marker.pose.position.z = 0.5
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1
        self.marker.scale.x = 0.6
        self.marker.scale.y = 0.6
        self.marker.scale.z = 1.0
        self.marker.color.r = 46. / 255.
        self.marker.color.g = 204. / 255.
        self.marker.color.b = 113. / 255.
        self.marker.color.a = 1.0

        self.text_marker.header.frame_id = "map"
        self.text_marker.header.stamp = rospy.Time.now()
        self.text_marker.type = Marker.TEXT_VIEW_FACING
        self.text_marker.pose.position.x = self.marker.pose.position.x
        self.text_marker.pose.position.y = self.marker.pose.position.y
        self.text_marker.pose.position.z = self.marker.pose.position.z + 1
        self.text_marker.scale.x = 0.3
        self.text_marker.scale.y = 0.3
        self.text_marker.scale.z = 0.3
        self.text_marker.color.r = 44. / 255.
        self.text_marker.color.g = 62. / 255.
        self.text_marker.color.b = 80. / 255.
        self.text_marker.color.a = 1.0
        self.text_marker.text = "Pedestrian"

        self.publisher.publish(self.marker)
        self.publisher_text.publish(self.text_marker)

    def set_goal(self, new_goal_position):
        """
        Sets the new goal of the pedestrian.

        :type new_goal_position: Vector3
        :param new_goal_position: The new goal position of the pedestrian.
        """
        self.goal = new_goal_position

    def get_current_goal(self):
        return self.goal


def get_velocity_cmd(msg, pedestrian):
    """
    Callback function for the velocity command of the pedestrian.
    :param msg: The velocity message
    :param pedestrian: The pedestrian object to assign the velocity
    """
    pedestrian.vel = msg


if __name__ == '__main__':
    rospy.init_node("pedestrian_node", anonymous=True)

    # Create Pedestrian
    pedestrian_obj = Pedestrian()

    # Subscriber for velocity command of the pedestrian
    rospy.Subscriber("cmd_vel_pedestrian", Twist, get_velocity_cmd, callback_args=pedestrian_obj)

    # Publisher for current position of the pedestrian
    # publisher_pedestrian_position = rospy.Publisher("pedestrian_position", Vector3, queue_size=10)

    # Create Path object with starting point
    path = Path(pedestrian_obj.start_position)

    # Setup the ini parser for getting publish rate
    ini_parser = IniParser()
    PUBLISH_RATE = int(ini_parser.get("Simulation")["publish_rate"])

    # Max velocity for pedestrian
    MAX_VEL = float(ini_parser.get("Pedestrian")["max_vel"])

    # Set the time
    last_time = rospy.Time().now()

    # Set publish rate
    rate = rospy.Rate(PUBLISH_RATE)

    while not rospy.is_shutdown():
        current_time = rospy.Time().now()
        dt = (current_time - last_time).to_sec()

        # Check if velocity is above MAX_VEL and clamp if so
        if pedestrian_obj.vel.linear.x >= MAX_VEL:
            pedestrian_obj.vel.linear.x = MAX_VEL
        if pedestrian_obj.vel.linear.y >= MAX_VEL:
            pedestrian_obj.vel.linear.y = MAX_VEL

        if pedestrian_obj.vel.linear.x <= -MAX_VEL:
            pedestrian_obj.vel.linear.x = -MAX_VEL
        if pedestrian_obj.vel.linear.y <= -MAX_VEL:
            pedestrian_obj.vel.linear.y = -MAX_VEL

        # Calculate the difference in position
        dx = pedestrian_obj.vel.linear.x * dt
        dy = pedestrian_obj.vel.linear.y * dt

        # Update the position
        pedestrian_obj.current_position.x += dx
        pedestrian_obj.current_position.y += dy

        # Publish the Markers visualization and positional information
        pedestrian_obj.publish()

        last_time = current_time

        rate.sleep()
