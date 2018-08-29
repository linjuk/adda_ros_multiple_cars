#!/usr/bin/python

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Point


def normalize(vec):
    x = vec[0] / np.linalg.norm(vec)
    y = vec[1] / np.linalg.norm(vec)
    z = vec[2] / np.linalg.norm(vec)

    return np.asarray([x, y, z])


class Path:
    def __init__(self):
        self.nodes = PoseArray()

    def path(self):
        return self.nodes

    def add_node(self, node):
        """
        :type node: Point
        """
        self.nodes.poses.append(node)

    def remove_node(self, node):
        self.nodes.poses.remove(node)

    def clear(self):
        for i in range(len(self.nodes.poses)):
            self.nodes.poses.pop()

    def size(self):
        return len(self.nodes.poses)

    def reverse(self):
        self.nodes.poses.reverse()

    def get_at(self, idx):
        if 0 <= idx < self.size():
            return self.nodes.poses[idx]
        return None

    def create_path_between_points(self, p1, p2, num_nodes, noise=0.0, seed=0):
        """
        Creates a path between two points with given number of nodes.
        :param p1: First point of the path
        :param p2: Last point of the path
        :param num_nodes: Number of nodes the path should have
        :type p1: Point
        :type p2: Point
        :type num_nodes: Integer
        """
        if num_nodes < 1:
            rospy.logerr("Nodes between paths can not be 0!")
            return False

        np.random.seed(seed)

        # Convert points into numpy arrays
        p1 = np.asarray([p1.x, p1.y, p1.z])
        p2 = np.asarray([p2.x, p2.y, p2.z])

        # Determine distance between every node
        direction = p2 - p1

        # Starting at the next point from the starting point, add nodes to the list
        for i in range(num_nodes+1):
            next_point = (direction * (float(i) / float(num_nodes))) + p1
            next_point[1] += np.random.normal(0, noise)
            self.add_node(Point(next_point[0], next_point[1], next_point[2]))

        rospy.loginfo("Got {} nodes on path!".format(num_nodes))
        return True

    def print_nodes(self):
        for node in self.nodes.poses:
            print "[{}, {}, {}]".format(node.x, node.y, node.z)
