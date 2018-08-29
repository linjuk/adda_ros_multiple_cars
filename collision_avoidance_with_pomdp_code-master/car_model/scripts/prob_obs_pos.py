#!/usr/bin/python

from scipy.stats import multivariate_normal

import numpy as np
import rospy
from car_model.srv import *


# ===================================================================================
# Global Variables
# ===================================================================================


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def transition_pedestrian(position, velocity, goal, dt):
    """
    The pedestrian model to compute the next position given the current position,
    the velocity and the goal. Returns a biased position vector.
    :param dt: Time difference.
    :param position: Current position of the pedestrian
    :param velocity: Current velocity of the pedestrian.
    :param goal: Goal position of the pedestrian.
    :return: New position.
    """
    change_in_position = (velocity * dt * normalize(goal - position))
    ret = position + change_in_position

    return ret


def cb_get_data(req):
    """
    Calculates the probability of the current observation being the calculated
    position with the pedestrian model.
    :param req: Last observation, current observation, goal and velocity.
    :return: Probability
    """
    # Convert Vector3 objects to numpy arrays
    last_observed_position = np.array(
        [req.last_observed_position.position.x, req.last_observed_position.position.y])
    goal = np.array([req.goal.x, req.goal.y])
    observed_position = np.array(
        [req.observed_position.position.x, req.observed_position.position.y])
    velocity = float(req.velocity)
    dt = req.dt

    # Change noise according to the type (trustworthy or not)
    if req.trust:
        PEDESTRIAN_NOISE = 0.02  # 0.05
    else:
        PEDESTRIAN_NOISE = 0.4  # 1.0

    # Calculate the next position given the last observed position, velocity and goal
    calculated_position = transition_pedestrian(last_observed_position, velocity, goal, dt)

    # Calculate prob. of new position being the current observation
    prob = multivariate_normal.pdf(observed_position, calculated_position,
                                   np.matrix([[PEDESTRIAN_NOISE, 0], [0, PEDESTRIAN_NOISE]]))

    return ProbObservedPositionResponse(prob)


if __name__ == '__main__':
    rospy.init_node("prob_observed_position", anonymous=True)
    rospy.Service("prob_obs_pos", ProbObservedPosition, cb_get_data)

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        rate.sleep()
