#!/usr/bin/python

"""
This file is responsible for managing the goals. This includes:
    - Visualizing the goals as Markers
    - Publishing the positions of the goals as Vector3
    - Publishing an Array of all available Goals as Vector3[]
"""

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from car_model.msg import Goals


class Goal:
    """
    Represents a Goal in rViz for the Pedestrian.
    """

    def __init__(self, position, publisher_marker, publisher_location):
        """
        Initializes a goal for the pedestrian.
        :param position: The static position of the goal as a Vector3
        :param publisher_marker: The publisher for the visual Marker
        :param publisher_location: The publisher for the location
        """
        self.goal_marker = Marker()
        self.position = position
        self.is_active_goal = False
        self.publisher_marker = publisher_marker
        self.publisher_location = publisher_location

        self.init_marker()

    def init_marker(self):
        self.goal_marker.header.frame_id = "map"
        self.goal_marker.ns = "goals"
        self.goal_marker.header.stamp = rospy.Time.now()
        self.goal_marker.type = Marker.SPHERE
        self.goal_marker.pose.position.x = self.position.x
        self.goal_marker.pose.position.y = self.position.y
        self.goal_marker.pose.position.z = 0.5
        self.goal_marker.pose.orientation.x = 0
        self.goal_marker.pose.orientation.y = 0
        self.goal_marker.pose.orientation.z = 0
        self.goal_marker.pose.orientation.w = 1
        self.goal_marker.scale.x = 1
        self.goal_marker.scale.y = 1
        self.goal_marker.scale.z = 1
        self.goal_marker.color.r = 0
        self.goal_marker.color.g = 0
        self.goal_marker.color.b = 1
        self.goal_marker.color.a = 1

        self.publish()

    def update_position(self, new_position):
        self.position = new_position

    def is_equal(self, position):
        """
        Checks whether this goals' position and the given position
        are equal, which means that they are the same.
        :param position: The given goal to test on.
        :return: True, if the position of the given goal is the same
                 as the goal of this object. False otherwise.
        """
        if position.x == self.position.x and position.y == self.position.y:
            return True
        return False

    def publish(self):
        """
        Publishes the visual Marker and the position of the goal.
        """
        self.publisher_marker.publish(self.goal_marker)
        self.publisher_location.publish(self.position)

    def set_active(self):
        """
        Sets the goal object as active and colors it red.
        """
        self.goal_marker.color.r = 1
        self.goal_marker.color.b = 0
        self.is_active_goal = True

    def set_inactive(self):
        """
        Sets the goal object as inactive and colors it blue.
        """
        self.goal_marker.color.r = 0
        self.goal_marker.color.b = 1
        self.is_active_goal = False


def cb_get_active_goal(msg, active_goal_position):
    active_goal_position.x = msg.x
    active_goal_position.y = msg.y


active_goal = Vector3(0, 0, 0)


def main():
    rospy.init_node("goals_marker", anonymous=True)
    rate = rospy.Rate(60)

    # Publisher for Marker and Location
    goal1_pub_marker = rospy.Publisher("goal1_marker", Marker, queue_size=100)
    goal2_pub_marker = rospy.Publisher("goal2_marker", Marker, queue_size=100)

    goal1_pub_position = rospy.Publisher("goal1_position", Vector3, queue_size=100)
    goal2_pub_position = rospy.Publisher("goal2_position", Vector3, queue_size=100)

    # Publisher for making available all locations of all goals (essential for the
    # pedestrian planner)
    goals_publisher = rospy.Publisher("pedestrian_goals", Goals, queue_size=100)

    # Subscriber from the pedestrian planner to activate current (real) goal
    rospy.Subscriber("active_goal", Vector3, cb_get_active_goal, callback_args=active_goal)

    # Define positions for goals
    goal1_position = Vector3(5.0, 27.0, 0.0)
    goal2_position = Vector3(19.0, 27.0, 0.0)

    # Define goals
    goal1 = Goal(goal1_position, goal1_pub_marker, goal1_pub_position)
    goal2 = Goal(goal2_position, goal2_pub_marker, goal2_pub_position)

    # Make an array of the positions of all goals
    goal_array = Goals()
    goal_array.goals.append(goal1_position)
    goal_array.goals.append(goal2_position)

    while not rospy.is_shutdown():
        # Check if the active goal is one of the defined goals and change color accordingly
        if goal1.is_equal(active_goal):
            goal1.set_active()
            goal2.set_inactive()
        elif goal2.is_equal(active_goal):
            goal2.set_active()
            goal1.set_inactive()

        # Publish all relevant information (Position + Marker)
        goal1.publish()
        goal2.publish()

        # Publish the locations of the goal as an Array
        goals_publisher.publish(goal_array)

        rate.sleep()


if __name__ == '__main__':
    main()
