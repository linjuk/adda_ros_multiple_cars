#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from path import *
import ini_parser
import random
import numpy as np
from geometry_msgs.msg import Vector3, Twist
from car_model.msg import Goals
from std_msgs.msg import Bool


class Container:
    def __init__(self, item):
        self.item = item


class Position:
    def __init__(self):
        self.position = np.array([0.0, 0.0, 0.0])


# =============================== Globals ===============================
pedestrian_position = Position()
start_planner = Bool()


# =============================== Callbacks ===============================
def cb_pedestrian_position(msg):
    """
    Callback for getting the pedestrian position.
    :type msg: Odometry
    """
    pedestrian_position.position[0] = msg.pose.pose.position.x
    pedestrian_position.position[1] = msg.pose.pose.position.y


def cb_get_all_goals(msg, container):
    container.item = msg.goals


def normalize(vec):
    x = vec[0] / np.linalg.norm(vec)
    y = vec[1] / np.linalg.norm(vec)
    z = vec[2] / np.linalg.norm(vec)
    return np.asarray([x, y, z])


def in_range(v1, v2, radius):
    distance = np.linalg.norm(v2 - v1)
    return distance <= radius


def cb_start_pedestrian_planner(msg):
    """
    Callback for getting the trigger to start the planner for the pedestrian planner.
    :param msg:
    :type msg: Bool
    :param start_planner:
    :return:
    """
    start_planner.data = msg.data


# =============================== Main ===============================

def main():
    rospy.init_node("pedestrian_planner", anonymous=True)

    # All goals of the pedestrian
    goals = Goals()
    container_goals = Container(goals)

    # Switch for starting the planner
    start_planner.data = False

    # =============================== Subscriber ===============================
    # Subscriber for position of the pedestrian
    rospy.Subscriber("pedestrian_position", Odometry, cb_pedestrian_position)

    # Subscriber for waiting for the start signal from the DESPOT
    rospy.Subscriber("start_pedestrian_planner", Bool, cb_start_pedestrian_planner)

    # Subscriber for getting all goal positions
    rospy.Subscriber("pedestrian_goals", Goals, cb_get_all_goals,
                     callback_args=container_goals)

    # =============================== Publisher ================================
    # Publisher for the current active goal
    active_goal_publisher = rospy.Publisher("active_goal", Vector3, queue_size=1)

    # Publisher for command velocity
    vel_publisher = rospy.Publisher("cmd_vel_pedestrian", Twist, queue_size=1)

    # =============================== Misc. ====================================
    # Ini parser for pedestrian properties from config file
    parser = ini_parser.IniParser()
    MAX_VEL = float(parser.get("Pedestrian")["max_vel"])
    PUBLISH_RATE = float(parser.get("Simulation")['publish_rate'])

    rospy.sleep(2)

    # Initially, pick a random goal from the list of goals.
    # idx = random.randint(0, len(container_goals.item) - 1)
    # current_goal = container_goals.item[idx]
    idx = 1
    current_goal = container_goals.item[idx]  # Pick second goal for now

    # Current velocity of the pedestrian
    velocity = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

    rate = rospy.Rate(PUBLISH_RATE)

    ##################
    NOISE = 1.5
    SEED = 1464254264
    path = Path()
    p1 = Point(pedestrian_position.position[0], pedestrian_position.position[1], 0.0)
    p2 = Point(current_goal.x, current_goal.y, 0.0)
    path.create_path_between_points(p1, p2, 20, noise=NOISE, seed=SEED)
    current_node_idx = 1
    ##################
    goal_changed = False
    while not rospy.is_shutdown():
        # Only start planner if DESPOT is ready
        if not start_planner.data:
            rate.sleep()
            continue

        # Store current goal as numpy array
        goal = np.array([current_goal.x, current_goal.y, 0])

        # Choose the next node if pedestrian is close enough
        target_node = np.asarray(
            [path.get_at(current_node_idx).x, path.get_at(current_node_idx).y, 0])
        if np.linalg.norm(target_node - pedestrian_position.position) < 1.0:
            current_node_idx += 1
            if current_node_idx >= path.size():
                path.reverse()
                current_node_idx = 0
                goal_changed = True

        # Calculate steering and new velocity for current node
        desired = normalize(target_node - pedestrian_position.position) * MAX_VEL
        velocity.linear.x += desired[0] - velocity.linear.x
        velocity.linear.y += desired[1] - velocity.linear.y

        # Change goal, if pedestrian reached the goal within a small range
        if goal_changed:
            goal_changed = False
            new_idx = idx
            while new_idx == idx:
                new_idx = random.randint(0, len(container_goals.item) - 1)
            current_goal = container_goals.item[new_idx]
            idx = new_idx

        # Publish the velocity and the active goal
        vel_publisher.publish(velocity)
        active_goal_publisher.publish(current_goal)

        rate.sleep()


if __name__ == '__main__':
    main()
