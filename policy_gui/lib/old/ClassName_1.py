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

import tf


class ClassName(QObject):
    _update_task_delegates = Signal()
    _update_endeffector_widget = Signal()

    def __init__(self, context):
        QObject.__init__(self, context)
        self.setObjectName('ClassName')



        # setup publisher
        self._examplePublisher = rospy.Publisher('/policy_gui/exampletopic', Bool,queue_size=10)

        self.goal_client = actionlib.SimpleActionClient('/car1/move_base', MoveBaseAction)
        self.goal_client.wait_for_server()

        self.velocity_service1_ = rospy.ServiceProxy('/car1/car_control/pomdp_velocity', ActionObservation)

        self.listener = tf.TransformListener()

        # car 2
        self._examplePublisher = rospy.Publisher('/policy_gui/exampletopic', Bool,queue_size=10)
        self.goal_client = actionlib.SimpleActionClient('/car2/move_base', MoveBaseAction)
        self.goal_client.wait_for_server()
        self.velocity_service2_ = rospy.ServiceProxy('/car2/car_control/pomdp_velocity', ActionObservation)
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


        #taurob_training_week = rospy.get_param('/taurob_training_week',False)

        # connect Signal Slot
        #self._update_task_delegates.connect(self._on_update_task_delegates)

        # init stuff
        self.bla = 1



        self.pos_car1 =[]
        self.pos_car2 =[]



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





    def compute_action(self, state_before):

        #compute action

        # STATE OBSERVATION 1:
        # print('Observing state before action:')
        # print('car position1:', self.pos_car1)
        # print('car position2:', self.pos_car2)
        #
        # distance_before = math.sqrt((self.pos_car1[0] - self.pos_car2[0]) ** 2 + (self.pos_car1[1] - self.pos_car2[1]) ** 2)
        # print('distance between cars: ', distance_before)

        state_before = np.zeros((10.1))
        state_before[0] = self.pos_car1[0]  # car1 x possition
        state_before[1] = self.pos_car1[1]  # car1 y possition
        state_before[2] = self.velocity_service1_.call(req)  # car1 velocity NOT SURE ABOUT THIS
        state_before[3] = goal.target_pose.pose.position.x  # car1 goal x NOT SURE ABOUT THIS
        state_before[4] = goal.target_pose.pose.position.y  # car1 goal y NOT SURE ABOUT THIS
        state_before[5] = self.pos_car2[0]  # car2 x possition
        state_before[6] = self.pos_car2[1]  # car2 y possition
        state_before[7] = self.velocity_service2_.call(req)  # car1 velocity NOT SURE ABOUT THIS
        state_before[8] = goal.target_pose.pose.position.x  # car1 goal x NOT SURE ABOUT THIS
        state_before[9] = goal.target_pose.pose.position.y  # car1 goal y NOT SURE ABOUT THIS


        print('Making an action:')
        actions = [3] * 65 + [4] * 10 + [2] * 10 + [1] * 15
        random_action = random.choice(actions)
        print(random_action)
        req = ActionObservationRequest()
        req.action = random_action
        res = self.velocity_service1_.call(req)

        time.sleep(3.0)

        return state_before, random_action

    def transition(self, state_before, action):
        #compute action
        # return observation + next state

        state_next = np.zeros((10.1))
        state_next[0] = self.pos_car1[0]  # car1 x possition
        state_next[1] = self.pos_car1[1]  # car1 y possition
        state_next[2] = self.velocity_service1_.call(req) # car1 velocity NOT SURE ABOUT THIS
        state_next[3] = goal.target_pose.pose.position.x # car1 goal x NOT SURE ABOUT THIS
        state_next[4] = goal.target_pose.pose.position.y  # car1 goal y NOT SURE ABOUT THIS
        state_next[5] = self.pos_car2[0]  # car2 x possition
        state_next[6] = self.pos_car2[1]  # car2 y possition
        state_next[7] = self.velocity_service2_.call(req)  # car1 velocity NOT SURE ABOUT THIS
        state_next[8] = goal.target_pose.pose.position.x  # car1 goal x NOT SURE ABOUT THIS
        state_next[9] = goal.target_pose.pose.position.y  # car1 goal y NOT SURE ABOUT THIS


        observation = np.zeros((7.1))
        observation[0] = self.pos_car1[0]  # car1 x possition
        observation[1] = self.pos_car1[1]  # car1 y possition
        observation[2] = self.velocity_service1_.call(req)  # car1 velocity NOT SURE ABOUT THIS
        observation[3] = self.pos_car2[0]  # car2 x possition
        observation[4] = self.pos_car2[1]  # car2 y possition
        observation[5] = self.velocity_service2_.call(req)  # car2 velocity NOT SURE ABOUT THIS
        observation[6] = math.sqrt((self.pos_car1[0] - self.pos_car2[0]) ** 2 + (self.pos_car1[1] - self.pos_car2[1]) ** 2)

        return state_next, observation

    def reward(self, state_next, action, observation=None):

        reward_total = 0
        print('Computing reward:')
        if observation[6] >= 10:
            reward = 5
        elif 10 > observation[6] >= 5:
            reward = 0
        elif 5 > observation[6] >= 2:
            reward = -10
        elif 2 > observation[6] >= 0:
            reward = -100

        reward_total += reward

        #print('Step reward: ', reward)
        #print('Total reward: ', reward_total)

        return reward, reward_total

        # compute action
        # return observation + next state








    def _on_setup_button_pressed(self):
        # should send two navigation goals
        print(' Setup Button pressed, publishing msg')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/car1/map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = 45.0
        goal.target_pose.pose.position.y = 17.0
        goal.target_pose.pose.orientation.z = 150.841592654
        self.goal_client.send_goal(goal)

        # moves only first stops

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/car2/map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = 33.0;
        goal.target_pose.pose.position.y = 45.0;
        goal.target_pose.pose.orientation.z = 20.441592654;
        self.goal_client.send_goal(goal)

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
        self.compute_policy()

        print (' Start Button pressed, publishing msg')

    def compute_policy(self):

        # random number generator with  3  for 50% of the times, 1 - 30%, 4 and 2 - both 10%
        # 1 - dec.; 2 - hold; 3 - acc.; 4 - stop
        goal_x = 45.0
        goal_y = 17.0


        while self.pos_car1[0] < goal_x and self.pos_car1[1] < goal_y:   # TODO: think about that + real time possition of cars + moving car2

           self.compute_action(self, state_before)
           self.transition(self, state_before, action)
           self.reward(self, state_next, action, observation=None)








        print ('Randomization is over')



        # make list of actions

        # 1 - dec.; 2 - hold; 3 - acc.; 4 - stop

        # actions = np.arange(20)
        #
        # actions[0] = 3
        # actions[1] = 3
        # actions[2] = 3
        # actions[3] = 3
        # actions[4] = 3
        # actions[5] = 3
        # actions[6] = 1
        # actions[7] = 1
        # actions[8] = 4
        # actions[9] = 4
        # actions[10] = 1
        # actions[11] = 1
        # actions[12] = 2
        # actions[13] = 3
        # actions[14] = 3
        # actions[15] = 1
        # actions[16] = 2
        # actions[17] = 2
        # actions[18] = 3
        # actions[19] = 3
        #
        #
        # for i in range(len(actions)):
        #     print (actions[i])
        #     req = ActionObservationRequest()
        #     req.action = actions[i]
        #     res = self.velocity_service1_.call(req)
        #     time.sleep(1.0)

        # print (len(actions))
        # print (' Setup Button pressed, publishing msg')