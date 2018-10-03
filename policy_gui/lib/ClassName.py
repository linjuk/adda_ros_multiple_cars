import os
import rospy
import rospkg
import math
import yaml
import roslib

import numpy as np
import time
import actionlib
import random
import math


from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QEvent, QModelIndex, QObject, Qt, QTimer, Signal, Slot
try:
    from python_qt_binding.QtGui import QShortcut, QTableWidgetItem, QWidget, QLCDNumber, QItemDelegate, QAbstractItemView
except ImportError:
    from python_qt_binding.QtWidgets import QShortcut, QTableWidgetItem, QWidget, QLCDNumber, QItemDelegate, QAbstractItemView

from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from pomdp_car_msgs.srv import ActionObservation, ActionObservationRequest, ActionObservationResponse
from geometry_msgs.msg import Pose

import tf


# class State:
#     # Represents a state in the POMDP.
#
#     def __init__(self):
#         self.car1_pose = Pose()
#         self.car2_pose = Pose()
#         self.car1_vel = 0.0
#         self.car2_vel = 0.0
#         self.car1_goal = Pose()
#         self.car2_goal = Pose()
#
# class Observation:
#     # Represents an Observation in the POMDP.
#
#     def __init__(self):
#         self.car1_pose = Pose()
#         self.car2_pose = Pose()
#         self.car1_vel = 0.0
#         self.car2_vel = 0.0
#         self.distance_between_cars = 0.0

class ClassName(QObject):
    _update_task_delegates = Signal()
    _update_endeffector_widget = Signal()

    def __init__(self, context):
        QObject.__init__(self, context)
        self.setObjectName('ClassName')

        self.reward_total = 0
        self.execute_policy = False
        self.t = 0

        self.pos_car1 = []
        self.pos_car2 = []


        self.car1_goal = [45., 17.]
        self.car2_goal = [33., 45.]

        # setup publisher
        self._examplePublisher = rospy.Publisher('/policy_gui/exampletopic', Bool,queue_size=10)

        self.goal_client1 = actionlib.SimpleActionClient('/car1/move_base', MoveBaseAction)
        self.goal_client1.wait_for_server()

        self.velocity_service1_ = rospy.ServiceProxy('/car1/car_control/pomdp_velocity', ActionObservation)
        self.position_service1_ = rospy.ServiceProxy('/car1/car_control/pomdp_position', ActionObservation)

        self.listener = tf.TransformListener()

        # car 2
        self._examplePublisher = rospy.Publisher('/policy_gui/exampletopic', Bool,queue_size=10)
        self.goal_client2 = actionlib.SimpleActionClient('/car2/move_base', MoveBaseAction)
        self.goal_client2.wait_for_server()
        self.velocity_service2_ = rospy.ServiceProxy('/car2/car_control/pomdp_velocity', ActionObservation)
        self.position_service2_ = rospy.ServiceProxy('/car2/car_control/pomdp_position', ActionObservation)
        self.listener = tf.TransformListener()


        # setup services
        #self.getObjects = rospy.ServiceProxy('worldmodel/get_object_model', GetObjectModel)

        # setup subscribers
        #self._worldmodelObjectsSubscriber = rospy.Subscriber("/worldmodel/objects", ObjectModel, self._on_worldmodel_objects)
        #self._interactive_marker_endeffector = rospy.Subscriber("/interactive_marker_pose_control/feedback",InteractiveMarkerFeedback,
                                                #self._on_interactive_marker_endeffector_pose,queue_size=1)


        # setup action clients
        #self._move_arm_client = actionlib.SimpleActionClient("/move_group", MoveGroupAction)

        #setup tf listener
        #self.tf_listener = TransformListener()

        # setup main widget
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('policy_gui'), 'lib', 'ClassName.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ClassNameUi')


        #set connections
        self._widget.start_button.pressed.connect(self._on_start_button_pressed)
        self._widget.setup_button.pressed.connect(self._on_setup_button_pressed)

        #getRobotJoints_button

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.


        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.timer = rospy.Timer(rospy.Duration(1), self.policy_loop)


        #taurob_training_week = rospy.get_param('/taurob_training_week',False)

        # connect Signal Slot
        #self._update_task_delegates.connect(self._on_update_task_delegates)

        # init stuff
        # self.bla = 1




    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

   # def _on_joint_states(self,message):
          #  arm_joints =['arm_joint_0', 'arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4']

    def random_action(self):
        actions = [3] * 65 + [2] * 10 + [1] * 15
        return random.choice(actions)


    def update_state(self):

        # TODO: at the moment this function uses initial values but it needs to be replaced with realtime values using the new service

        # fetch values from service
        # update self.pos_car1 and 2
        # self.pos_car1 index => 0 = x, 1 = y, 2 = velocity, 3 = target.x, 4 = target.y

        time_now = rospy.Time(0)
        (trans1, rot1) = self.listener.lookupTransform('/map', '/car1/base_link', time_now)
        (trans2, rot2) = self.listener.lookupTransform('/map', '/car2/base_link', time_now)

        self.pos_car1 = trans1
        self.pos_car2 = trans2

        state = np.zeros((10,1))
        state[0] = self.pos_car1[0]  # car1 x possition
        state[1] = self.pos_car1[1]  # car1 y possition
        # state[2] = self.velocity_service1_.call(req)  # car1 velocity NOT SURE ABOUT THIS
        # state[3] = goal.target_pose.pose.position.x  # car1 goal x NOT SURE ABOUT THIS
        # state[4] = goal.target_pose.pose.position.y  # car1 goal y NOT SURE ABOUT THIS
        state[5] = self.pos_car2[0]  # car2 x possition
        state[6] = self.pos_car2[1]  # car2 y possition
        # state[7] = self.velocity_service2_.call(req)  # car1 velocity NOT SURE ABOUT THIS
        # state[8] = goal.target_pose.pose.position.x  # car1 goal x NOT SURE ABOUT THIS
        # state[9] = goal.target_pose.pose.position.y  # car1 goal y NOT SURE ABOUT THIS

        return state




    def compute_action(self, state):

        print('Compute action ...')

        # TODO: decide the logic to determine the action based on state, right now its random action with weighted options

        print('Generating a random action:')
        action = self.random_action()
        print(action)

        return state, action



    def transition(self, state, action):

        print('Transition ...')

        # perform action suggested by compute_action
        req = ActionObservationRequest()
        req.action = action
        res = self.velocity_service1_.call(req)

        # wait for 3 seconds
        # time.sleep(3.0)

        # get new state
        new_state = self.update_state()

        # observe environment
        observation = np.zeros((7, 1))
        observation[0] = self.pos_car1[0]  # car1 x possition
        observation[1] = self.pos_car1[1]  # car1 y possition
        # observation[2] = self.velocity_service1_.call(req)  # car1 velocity NOT SURE ABOUT THIS
        observation[3] = self.pos_car2[0]  # car2 x possition
        observation[4] = self.pos_car2[1]  # car2 y possition
        # observation[5] = self.velocity_service2_.call(req)  # car2 velocity NOT SURE ABOUT THIS


        return new_state, observation

    def reward(self, state, action, observation):

        # Negative reward for the action that was made ( for all actions the same negative reward)
        # Negative reward, if the car is to close to the other car (pre-defined range)
        # Speed reward ???
        # Positive reward, if the car is close to its goal (pre-defined range)

        print('Computing reward:')
        print('Observation.distance: ', observation[6])

        print("car 1: ")
        print(self.pos_car1)
        print("car 2: ")
        print(self.pos_car2)

        dist_car = math.sqrt( (state[0] - state[5]) ** 2 + (state[1] - state[6]) ** 2)
        dist_goal = math.sqrt((state[0] - self.car1_goal[0]) ** 2 + (state[1] - self.car1_goal[1]) ** 2)

        print("distance", dist_car)
        print('goal distance ', dist_goal)
        if dist_car >= 10:
            reward = 5
        elif 10 > dist_car >= 5:
            reward = 0
        elif 5 > dist_car >= 2:
            reward = -10
        elif 2 > dist_car >= 0:
            reward = -100

        #print('Step reward: ', reward)
        #print('Total reward: ', reward_total)

        return reward

        # compute action
        # return observation + next state








    def _on_setup_button_pressed(self):
        # should send two navigation goals
        print(' Setup Button pressed, publishing msg')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/car1/map"
        goal.target_pose.header.stamp = rospy.Time.now()


        goal.target_pose.pose.position.x = self.car1_goal[0]
        goal.target_pose.pose.position.y = self.car1_goal[1]
        goal.target_pose.pose.orientation.z = 150.841592654
        self.goal_client1.send_goal(goal)

        # moves only first stops

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/car2/map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = self.car2_goal[0]
        goal.target_pose.pose.position.y = self.car2_goal[1]
        goal.target_pose.pose.orientation.z = 20.441592654
        self.goal_client2.send_goal(goal)

        time_now =rospy.Time(0)
        (trans1, rot1) = self.listener.lookupTransform('/car1/base_link', '/map',time_now )
        (trans2, rot2) = self.listener.lookupTransform('/car2/base_link', '/map', time_now)

        self.pos_car1 = trans1
        self.pos_car2 = trans2

        print( "car 1: " )
        print (self.pos_car1)
        print("car 2: ")
        print(self.pos_car2)

    def _on_start_button_pressed(self):
        # should call compute policy method and compute policy will give list of actions.
        # Should execute one action after another (kinda loop). Before or together send
        #  velocity to another car
        #self.compute_policy()
        self.execute_policy = True
        self.reward_total = 0
        self.t = 0

        req = ActionObservationRequest()
        req.action = 3
        res = self.velocity_service2_.call(req)

        print (' Start Button pressed, publishing msg')


    def policy_loop(self, event):
        goal_x = 45.0
        goal_y = 17.0


        if self.execute_policy:
            self.t = self.t + 1
            print('EXECUTING POLICY ', self.t)
            current_state = self.update_state()
            state_before_transition, action = self.compute_action(current_state)
            state_next_transition, observation = self.transition(state_before_transition, action)
            print('Reward total: ', self.reward_total)
            iteration_reward = self.reward(state_next_transition, action, observation)
            print('Iteration reward total: ', iteration_reward)
            self.reward_total += iteration_reward
            if  self.pos_car1[0] >= goal_x and self.pos_car1[1] >= goal_y:
                self.execute_policy=False

    # def compute_policy(self):

        # random number generator with  3  for 50% of the times, 1 - 30%, 4 and 2 - both 10%
        # 1 - dec.; 2 - hold; 3 - acc.; 4 - stop


        # while self.pos_car1[0] < goal_x and self.pos_car1[1] < goal_y:   # TODO: think about that + real time possition of cars + moving car2
        #
        #     current_state = self.update_state()
        #     state_before_transition, action = self.compute_action(current_state)
        #     state_next_transition, observation = self.transition(state_before_transition, action)
        #     print('Reward total: ', reward_total)
        #     iteration_reward = self.reward(state_next_transition, action, observation)
        #     print('Iteration reward total: ', iteration_reward)
        #     reward_total += iteration_reward
        #
        #     stop condition for the loop
        #     if(x===y) break
        #
        # print ('Total reward:', reward_total)
        # print ('Randomization is over')
#