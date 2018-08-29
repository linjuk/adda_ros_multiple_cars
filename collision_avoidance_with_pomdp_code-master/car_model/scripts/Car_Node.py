#!/usr/bin/python
import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Pose, Point, Twist, Vector3
from car_model.srv import *
import ConfigParser


# Ini file parser
config = ConfigParser.ConfigParser()
config.read("/home/albert/ias_ros/src/car_model/config/config.ini")


class CarState:
    def __init__(self, car_x, car_y, ped_x, ped_y, car_v, th):
        self.car_x = car_x
        self.car_y = car_y
        self.ped_x = ped_x
        self.ped_y = ped_y
        self.car_v = car_v
        self.th = th


class VelocityGetter:
    """ A simple class for storing the incoming velocity commands. """

    def __init__(self):
        self.vx = 0.0
        self.vy = 0.0
        self.vh = 0.0


class ObservationGetter:
    def __init__(self):
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_v = 0.0
        self.ped_x = 0.0
        self.ped_y = 0.0


class ActionSender:
    def __init__(self):
        self.action = 1  # default is hold
        self.action_ready = False
        self.observation = CarState(0, 0, 0, 0, 0, 0)

    def get_observation(self):
        return self.observation

    def set_observation(self, state):
        self.observation = state


def in_range(a, b, r):
    distance = np.sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2))
    print "Distance: ", distance
    return distance <= r


action_sender = ActionSender()
vg = VelocityGetter()
pedestrian_position = Vector3()


def get_acceleration(msg, vg):
    """ Callback method for getting the angular velocity as a Twist message. Only needed for the path planner.
        :param msg: the Twist message itself.
        :param vg: the velocity wrapper for storing the incoming velocity value so they can be used in this node.
    """
    vg.vh = msg.angular.z


_x = []
_y = []

def handle_request(req):
    """
    Request is an action (0, 1 or 2). Based on the number, the \cmd_vel will get
    a new velocity command. After the command is executed, the simulation returns
    the observation encoded as an uint64_t.
    3 will be used for stopping the car immediately.
    :param req: THe requested action from DESPOT
    :return: THe response from the observation
    """
    if req.action not in [0, 1, 2, 3]:
        rospy.ERROR("Wrong ActionObservation Request received: ", req.action)
        return

    if req.action == 3:
        vg.vx = 0
        return

    # Mark the given action as valid
    action_sender.action = req.action
    action_sender.action_ready = True

    # Get the current observation
    obs = action_sender.get_observation()
    x = obs.car_x
    y = obs.car_y
    th = obs.th

    _x.append(x)
    _y.append(y)

    last_time = rospy.Time.now()

    # Get the real observation by calculating the "next" state
    # obs2 = _calculate_next_state(x, y, th, last_time)

    # Set all response variables
    response = ActionObservationResponse()
    response.car_pose.position.x = x
    response.car_pose.position.y = y
    response.car_pose.orientation.z = th
    response.pedestrian_position.position.x = obs.ped_x
    response.pedestrian_position.position.y = obs.ped_y

    response.car_v = vg.vx
    # response.car_pose.position.x = obs2[0]
    # response.car_pose.position.y = obs2[1]
    # response.car_pose.orientation.z = obs2[3]
    # response.car_velocity = obs2[2]
    # response.pedestrian_position.x = obs.ped_x
    # response.pedestrian_position.y = obs.ped_y
    # response.car_x = x  # obs2[0]
    # response.car_y = y  # obs2[1]
    # response.ped_x = obs.ped_x
    # response.ped_y = obs.ped_y
    # response.car_v = vg.vx  # obs2[2]
    # response.theta = th  # obs2[3]

    return response


def ConfigSectionMap(config, section):
    """
        Helper function for the *ini file reader.
        @param config :     The configurator object.
        @param section :    Section name in the *ini file.
        @return :   A dictionary of the sections' parameters.
    """
    dict1 = {}
    options = config.options(section)
    for option in options:
        try:
            dict1[option] = config.get(section, option)
            if dict1[option] == -1:
                print("skip: %s" % option)
        except:
            print("exception on %s!" % option)
            dict1[option] = None
    return dict1


def get_pedestrian_position(msg):
    pedestrian_position.x = msg.pose.pose.position.x
    pedestrian_position.y = msg.pose.pose.position.y


# Get parameters from the config
MAX_VEL = float(ConfigSectionMap(config, "Car")['max_vel'])
PUBLISH_RATE = float(ConfigSectionMap(config, "Simulation")['publish_rate'])
ACCELERATION = float(ConfigSectionMap(config, "Car")['acceleration'])


def main():
    rospy.init_node('car')
    rospy.loginfo("\033[1;32mStarting Car Node !\033[0m\n")

    # Service for exchanging action/observation with POMDP / DESPOT
    rospy.Service('info_exchanger', ActionObservation, handle_request)

    # Publisher for Odometry information
    odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=10)

    # Subscriber for incoming velocity commands
    rospy.Subscriber('/cmd_vel', Twist, get_acceleration, callback_args=vg)

    # Subscriber for Pedestrian Position
    rospy.Subscriber("pedestrian_position", Odometry, get_pedestrian_position)

    # TF Broadcaster
    odom_broadcaster = tf.TransformBroadcaster()

    # Get the starting position for the car from the configuration file.
    x = float(ConfigSectionMap(config, "Car")['start_x'])
    y = float(ConfigSectionMap(config, "Car")['start_y'])
    th = np.pi / 2.0

    last_time = rospy.Time.now()
    rate = rospy.Rate(PUBLISH_RATE)

    # Initialize transformation and odometry
    odom_trans = TransformStamped()
    odom_trans.header.frame_id = 'map'  # From map...
    odom_trans.child_frame_id = 'base_link'  # To the car's base

    odom = Odometry()
    odom.header.frame_id = 'map'
    odom.child_frame_id = 'base_link'

    # Main loop
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # Handle the velocity command from the DESPOT. First, check if we can accept a new
        # velocity command.
        if action_sender.action_ready:
            if action_sender.action == 0:
                # Don't exceed maximum velocity
                if vg.vx <= MAX_VEL - ACCELERATION:
                    vg.vx += ACCELERATION
            elif action_sender.action == 2:
                # For now, don't backup the car
                if vg.vx >= 0.0 + ACCELERATION:
                    vg.vx -= ACCELERATION

        # "Close" the velocity command receiver to prevent false actions.
        action_sender.action_ready = False

        # Calculate changed velocity by integrating over time step dt
        dt = (current_time - last_time).to_sec()
        dx = (vg.vx * np.cos(th)) * dt
        dy = (vg.vx * np.sin(th)) * dt
        dh = vg.vh * dt

        # Update the new position
        x += dx
        y += dy
        th += dh

        # Quaternion for the angular position
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # TF Broadcaster Setup
        odom_trans.header.stamp = current_time

        # Gather all information about the current position
        odom_trans.transform.translation = (x, y, 0.0)
        odom_trans.transform.rotation = odom_quat

        # Broadcast the TF
        odom_broadcaster.sendTransform(odom_trans.transform.translation, odom_trans.transform.rotation,
                                       odom_trans.header.stamp, odom_trans.child_frame_id, odom_trans.header.frame_id)

        # Odometry Publisher Setup
        odom.header.stamp = current_time

        # Gather positional and velocity information
        odom.pose.pose = Pose(Point(x, y, 0.0), Quaternion(*odom_quat))
        odom.twist.twist = Twist(Vector3(vg.vx, 0.0, 0.0), Vector3(0, 0, vg.vh))
        odom_publisher.publish(odom)

        last_time = current_time

        # Make a "state" for the observation to transmit it to the DESPOT if needed.
        current_state = CarState(x, y, pedestrian_position.x, pedestrian_position.y, vg.vx, th)
        action_sender.set_observation(current_state)

        rate.sleep()

    plt.xlim([0, 50])
    plt.ylim([0, 50])
    plt.plot(_x, _y, ".")
    plt.grid()
    # plt.show()


def _calculate_next_state(x, y, th, last_time):
    """
        This function will calculate the next state for the car given its current
        configuration. This is needed for the right observation, because without
        this calculation, the car will observe the environment from the last time
        step.
    """
    current_time = rospy.Time.now()
    # Save the current velocity in order to not destroy the real velocity value vg.vx
    vel = vg.vx
    vel_vh = vg.vh

    if action_sender.action == 0:
        if not vel >= MAX_VEL:
            vel += ACCELERATION
    elif action_sender.action == 2:
        if not vel <= 0:
            vel -= ACCELERATION

    # Calculate changed velocity by integrating over timestep dt
    dt = (current_time - last_time).to_sec()
    dx = (vel * np.cos(th)) * dt
    dy = (vel * np.sin(th)) * dt
    dh = vel_vh * dt

    # Update the new position
    x += dx
    y += dy
    th += dh

    return [x, y, vel, th]


if __name__ == '__main__':
    main()
