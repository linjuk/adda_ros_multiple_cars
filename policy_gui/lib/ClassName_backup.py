import os
import rospy
import rospkg
import math
import yaml
import roslib
import csv

import matplotlib.pyplot as plt
plt.style.use("seaborn")
from numpy import genfromtxt

import pandas as pd
import numpy as np
import time
import actionlib
import random
import math
import tf
import pickle


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


# noinspection PyInterpreter
class ClassName(QObject):
    _update_task_delegates = Signal()
    _update_endeffector_widget = Signal()

    def __init__(self, context):
        QObject.__init__(self, context)
        self.setObjectName('ClassName')

        self.reward_total = 0
        self.execute_policy = False
        self.record_button = False
        self.save_button = False
        self.plot_button = False
        self.t = 0

        self.pos_car1 = []
        self.pos_car2 = []
        self.trajectory_collector = []

        self.listener = tf.TransformListener()


        self.car1_goal = [38., 3.]
        #self.car2_goal = [38., 3.]
        self.car2_goal = [30., 32.]

        # setup publisher
        # car 1
        self._examplePublisher = rospy.Publisher('/policy_gui/exampletopic', Bool,queue_size=10)
        self.goal_client1 = actionlib.SimpleActionClient('/car1/move_base', MoveBaseAction)
        self.goal_client1.wait_for_server()
        self.velocity_service1_ = rospy.ServiceProxy('/car1/car_control/pomdp_velocity', ActionObservation)
        #self.listener = tf.TransformListener()

        # car 2
        self._examplePublisher = rospy.Publisher('/policy_gui/exampletopic', Bool,queue_size=10)
        self.goal_client2 = actionlib.SimpleActionClient('/car2/move_base', MoveBaseAction)
        self.goal_client2.wait_for_server()
        self.velocity_service2_ = rospy.ServiceProxy('/car2/car_control/pomdp_velocity', ActionObservation)
        # self.listener = tf.TransformListener()


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
        self._widget.record_button.pressed.connect(self._on_record_button_pressed)
        self._widget.save_button.pressed.connect(self._on_save_button_pressed)
        self._widget.plot_button.pressed.connect(self._on_plot_button_pressed)

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

    ############################################################
    ######################### BUTTONS ##########################
    ############################################################

    ###################################
    # Setup button (Set Goal to Cars) #
    ###################################

    def _on_setup_button_pressed(self):
        # should send two navigation goals
        print(' Setup Button pressed, publishing msg')

        # goal for first car
        goal1 = MoveBaseGoal()
        goal1.target_pose.header.frame_id = "/car1/map"
        goal1.target_pose.header.stamp = rospy.Time.now()
        goal1.target_pose.pose.position.x = self.car1_goal[0]
        goal1.target_pose.pose.position.y = self.car1_goal[1]
        goal1.target_pose.pose.orientation.z = 1.0
        self.goal_client1.send_goal(goal1)

        # goal for second car
        goal2 = MoveBaseGoal()
        goal2.target_pose.header.frame_id = "/car2/map"
        goal2.target_pose.header.stamp = rospy.Time.now()
        goal2.target_pose.pose.position.x = self.car2_goal[0]
        goal2.target_pose.pose.position.y = self.car2_goal[1]
        goal2.target_pose.pose.orientation.z = 1.0
        self.goal_client2.send_goal(goal2)

        time_now = rospy.Time(0)
        (trans1, rot1) = self.listener.lookupTransform('/car1/base_link', '/map', time_now)
        (trans2, rot2) = self.listener.lookupTransform('/car2/base_link', '/map', time_now)

        self.pos_car1 = trans1
        self.pos_car2 = trans2

        print("car 1: ")
        print(self.pos_car1)
        print("car 2: ")
        print(self.pos_car2)

        #####################################################
        # Compute policy button (action/observarion/reward) #
        #####################################################

    def _on_start_button_pressed(self):
        # should call compute policy method and compute policy will give list of actions.
        # Should execute one action after another (kinda loop). Before or together send
        #  velocity to another car
        # self.compute_policy()
        self.execute_policy = True
        self.reward_total = 0
        self.t = 0

        req = ActionObservationRequest()
        req.action = 5
        res = self.velocity_service2_.call(req)

        #print(' Start Button pressed, publishing msg')

    ########################
    # Trajectory Recording #
    ########################

    def _on_record_button_pressed(self):
        # Should record trajectory
        print('Recording trajectory')
        # position = []
        self.record_button = True
        self.save_button = False
        self.trajectory_collector = []
        self.timer_position = rospy.Timer(rospy.Duration(1.0 / 30.0), self.trajectory_recording)

    #################################
    # Trajectory Saving to the File #
    #################################

    def _on_save_button_pressed(self):
        # Should record trajectory
        self.record_button = False
        self.save_button = True
        print('Saving data to the file...')

        path = "/home/linjuk/adda_ros/src/code_lina/trajectories"
        if not os.path.exists(path):
            os.makedirs(path)

        self.save_positions(path)
        self.save_button = False
        self.trajectory_collector = []

        # self.seq = self.trajectory_collector
        # self.file = open(os.path.join(path, "linaFile.txt"), "w")
        # self.file.write(self.trajectory_collector)

    #######################
    # Plotting Trajectory #
    #######################

    def _on_plot_button_pressed(self):
        # Should plot trajectories

        self.plot_button = True
        print('Plotting Trajectories...')

        print('straight')
        # straight_trajectory_files = [
        #     '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_1.csv',
        #     '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_2.csv',
        #     '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_3.csv',
        #     '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_4.csv',
        #     '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_5.csv',
        # ]
        #
        # straight_master = []
        # for file in straight_trajectory_files:
        #     print(len(self.calculate_time_step_average(file)))
        #     straight_master.append(self.calculate_time_step_average(file))
        #
        #
        # straight_master = pd.DataFrame[straight_master]
        # print(straight_master)

        # straight_master.to_csv('/home/linjuk/adda_ros/src/code_lina/trajectories/100_time_steps_{time}.csv'.format(time = abs(time.time())), sep=',', encoding='utf-8')

        print(self.calculate_time_step_average('/home/linjuk/adda_ros/src/code_lina/trajectories/forward_1.csv'))
        print(self.calculate_time_step_average('/home/linjuk/adda_ros/src/code_lina/trajectories/forward_2.csv'))
        print(self.calculate_time_step_average('/home/linjuk/adda_ros/src/code_lina/trajectories/forward_3.csv'))
        print(self.calculate_time_step_average('/home/linjuk/adda_ros/src/code_lina/trajectories/forward_4.csv'))
        print(self.calculate_time_step_average('/home/linjuk/adda_ros/src/code_lina/trajectories/forward_5.csv'))
        print('left')
        print(self.calculate_time_step_average('/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_1.csv'))
        print(self.calculate_time_step_average('/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_2.csv'))
        print(self.calculate_time_step_average('/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_3.csv'))
        print(self.calculate_time_step_average('/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_4.csv'))
        print(self.calculate_time_step_average('/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_5.csv'))
        print('right')
        print(self.calculate_time_step_average('/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_1.csv'))
        print(self.calculate_time_step_average('/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_2.csv'))
        print(self.calculate_time_step_average('/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_3.csv'))
        print(self.calculate_time_step_average('/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_4.csv'))
        print(self.calculate_time_step_average('/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_5.csv'))


        file_path = '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_4.csv'

        data = []
        with open(file_path, 'r') as f:
            reader = csv.reader(f, delimiter=',')

            # skip header (time, x, y, etc.)
            next(reader)
            for row in reader:
                element = []
                for entry in row:
                    element.append(float(entry))
                data.append(element)
            return np.asarray(data)

    ############ 0 PLOT ##############

#    numline = len(file_read.readlines())
#     row_count = 0
#     with open('/home/linjuk/adda_ros/src/code_lina/trajectories/forward_1.csv', 'r') as f:
#         reader = csv.reader(f, delimiter=',')
#         row_count = sum(1 for row in reader)
#         row_count -= 1
#
#     with open('/home/linjuk/adda_ros/src/code_lina/trajectories/forward_1.csv', 'r') as f:
#         reader = csv.reader(f, delimiter=',')
#         required_rows = 100
#         bucket = row_count / required_rows
#
#         print('Row count')
#         print(row_count)
#         print('Bucket')
#         print(bucket)
#
#         # next(reader)
#
#         counter = 1
#         values_of_interest = 0
#         averaged_results = []
#         for row in reader:
#             if counter % bucket == 0:
#                 values_of_interest += 1
#                 list(reader)[counter]
#             counter+=1
#         print('Values we are interested in')
#         print(values_of_interest)

    def calculate_time_step_average(self, file):
        df = pd.read_csv(file)
        # print(df)
        row_count = len(df)
        required_rows = 100
        bucket = int(round(row_count / required_rows))

        counter = 0
        averaged_x = []
        averaged_y = []
        averaged_z = []
        # aggregate = []

        if bucket >= 2:
            for index, row in df.iterrows():
                if(index % bucket == 0):
                    averaged_x.append(df[counter:counter+bucket]['x'].mean())
                    averaged_y.append(df[counter:counter + bucket]['y'].mean())
                    averaged_z.append(df[counter:counter + bucket]['z'].mean())
                    # aggregate.append([df[counter:counter + bucket]['x'].mean(),df[counter:counter + bucket]['y'].mean(),df[counter:counter + bucket]['z'].mean()])
                    counter += bucket
            # averaged_x = averaged_x[-100:]
            # averaged_y = averaged_y[-100:]
            # averaged_z = averaged_z[-100:]
        else:
            averaged_x = df['x']
            averaged_y = df['y']
            averaged_z = df['z']
            # aggregate = df



        return averaged_x, averaged_y, averaged_z
        # return len(averaged_x), len(averaged_y), len(averaged_z)

        # averaged_xr.to_csv('/home/linjuk/adda_ros/src/code_lina/trajectories/100_time_steps_{time}.csv'.format(time = abs(time.time())), sep=',', encoding='utf-8')

        # path = "/home/linjuk/adda_ros/src/code_lina/trajectories/100"
        # file_to_save = df.to_csv("100_time_steps.csv".format(path, num("%d", "w")))












    ############ 1st PLOT ##############
    # files = [
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_1.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_2.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_3.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_4.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_5.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_1.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_2.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_3.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_4.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_5.csv',
    #
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_1.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_2.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_3.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_4.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_5.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_1.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_2.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_3.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_4.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_5.csv',
    #
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/back_1.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/back_2.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/back_3.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/back_4.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/back_5.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_1.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_2.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_3.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_4.csv',
    #     '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_5.csv'
    # ]

    # file dictionary for easy file management and looping
    files_dictionary = {
        'left': [
            '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_1.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_2.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_3.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_4.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_5.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_1.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_2.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_3.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_4.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_5.csv',
        ],
        'right': [
            '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_1.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_2.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_3.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_4.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_5.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_1.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_2.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_3.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_4.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_5.csv',
        ],
        'straight': [
            '/home/linjuk/adda_ros/src/code_lina/trajectories/back_1.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/back_2.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/back_3.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/back_4.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/back_5.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_1.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_2.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_3.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_4.csv',
            '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_5.csv'
        ],
    }

    # container dictionary for ploting graphs
    trajectory_csv_data = {}

    # container dictionary for averaging
    trajectory_csv_file_wise_data = {}

    # color map for graph
    color_map = {
        'left': 'green',
        'right': 'blue',
        'straight': 'magenta'
    }

    # plot first graph
    plt.figure(1)
    plt.title('Movement Clasification f = y(x)')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')

    # labels
    plt.plot(0, 0, color='green', label='Left-Side Parking')
    plt.plot(0, 0, color='blue', label='Right-Side Parking')
    plt.plot(0, 0, color='magenta', label='Straight-Side Parking')

    # loop over trajectories i.e. left, right, straight
    for key in files_dictionary:
        trajectory_csv_file_wise_data[key] = {}
        # loop over files in each trajectory
        for index, file in enumerate(files_dictionary[key]):
            # read file
            trajectory_csv_data[key] = (genfromtxt(file, dtype=float, delimiter=",", skip_header=1))
            # aggregate data in container for averaging
            trajectory_csv_file_wise_data[key][index] = []
            trajectory_csv_file_wise_data[key][index].append(trajectory_csv_data[key])
            # segregate x and y for plotting
            x, y = trajectory_csv_data[key][:, 1], trajectory_csv_data[key][:, 2]
            p = plt.plot(x, y, color=color_map[key])

    plt.legend(loc='lower right')
    plt.show()

    row = []
    for index, file in enumerate(files_dictionary['left']):
        for rows in enumerate(file[0]):
            row.append(trajectory_csv_file_wise_data['left'][index][0][rows])

    print(row)
    # print(np.mean(row1, axis=0))


    # data = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_1.csv', dtype=float, delimiter=",")
    # data2 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_2.csv', dtype=float, delimiter=",")
    # data3 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_3.csv', dtype=float, delimiter=",")
    # data4 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_4.csv', dtype=float, delimiter=",")
    # data5 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_5.csv', dtype=float, delimiter=",")
    # data6 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_1.csv', dtype=float, delimiter=",")
    # data7 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_2.csv', dtype=float, delimiter=",")
    # data8 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_3.csv', dtype=float, delimiter=",")
    # data9 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_4.csv', dtype=float, delimiter=",")
    # data10 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_5.csv', dtype=float, delimiter=",")
    #
    #
    # data11 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_1.csv', dtype=float, delimiter=",")
    # data12 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_2.csv', dtype=float, delimiter=",")
    # data13 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_3.csv', dtype=float, delimiter=",")
    # data14 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_4.csv', dtype=float, delimiter=",")
    # data15 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_5.csv', dtype=float, delimiter=",")
    #
    # data16 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_1.csv', dtype=float, delimiter=",")
    # data17 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_2.csv', dtype=float, delimiter=",")
    # data18 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_3.csv', dtype=float, delimiter=",")
    # data19 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_4.csv', dtype=float, delimiter=",")
    # data20 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_5.csv', dtype=float, delimiter=",")
    #
    #
    # data21 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/back_1.csv', dtype=float, delimiter=",")
    # data22 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/back_2.csv', dtype=float, delimiter=",")
    # data23 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/back_3.csv', dtype=float, delimiter=",")
    # data24 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/back_4.csv', dtype=float, delimiter=",")
    # data25 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/back_5.csv', dtype=float, delimiter=",")
    #
    # data26 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/forward_1.csv', dtype=float, delimiter=",")
    # data27 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/forward_2.csv', dtype=float, delimiter=",")
    # data28 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/forward_3.csv', dtype=float, delimiter=",")
    # data29 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/forward_4.csv', dtype=float, delimiter=",")
    # data30 = genfromtxt('/home/linjuk/adda_ros/src/code_lina/trajectories/forward_5.csv', dtype=float, delimiter=",")
    #
    # x = data[:, 1]
    # x2 = data2[:, 1]
    # x3 = data3[:, 1]
    # x4 = data4[:, 1]
    # x5 = data5[:, 1]
    # x6 = data6[:, 1]
    # x7 = data7[:, 1]
    # x8 = data8[:, 1]
    # x9 = data9[:, 1]
    # x10 = data10[:, 1]
    # x11 = data11[:, 1]
    # x12 = data12[:, 1]
    # x13 = data13[:, 1]
    # x14 = data14[:, 1]
    # x15 = data15[:, 1]
    # x16 = data16[:, 1]
    # x17 = data17[:, 1]
    # x18 = data18[:, 1]
    # x19 = data19[:, 1]
    # x20 = data20[:, 1]
    # x21 = data21[:, 1]
    # x22 = data22[:, 1]
    # x23 = data23[:, 1]
    # x24 = data24[:, 1]
    # x25 = data25[:, 1]
    # x26 = data26[:, 1]
    # x27 = data27[:, 1]
    # x28 = data28[:, 1]
    # x29 = data29[:, 1]
    # x30 = data30[:, 1]
    #
    # y = data[:, 2]
    # y2 = data2[:, 2]
    # y3 = data3[:, 2]
    # y4 = data4[:, 2]
    # y5 = data5[:, 2]
    # y6 = data6[:, 2]
    # y7 = data7[:, 2]
    # y8 = data8[:, 2]
    # y9 = data9[:, 2]
    # y10 = data10[:, 2]
    # y11 = data11[:, 2]
    # y12 = data12[:, 2]
    # y13 = data13[:, 2]
    # y14 = data14[:, 2]
    # y15 = data15[:, 2]
    # y16 = data16[:, 2]
    # y17 = data17[:, 2]
    # y18 = data18[:, 2]
    # y19 = data19[:, 2]
    # y20 = data20[:, 2]
    # y21 = data21[:, 2]
    # y22 = data22[:, 2]
    # y23 = data23[:, 2]
    # y24 = data24[:, 2]
    # y25 = data25[:, 2]
    # y26 = data26[:, 2]
    # y27 = data27[:, 2]
    # y28 = data28[:, 2]
    # y29 = data29[:, 2]
    # y30 = data30[:, 2]
    #
    # # plot(x, y, color='green', linestyle='dashed', marker='o', markerfacecolor='blue', markersize=12).
    #
    # plt.figure(2)
    # plt.title('Movement Clasification f = y(x)')
    # plt.xlabel('x [m]')
    # plt.ylabel('y [m]')
    # plt.plot(x, y, color='green', label='Left-Side Parking')
    # plt.plot(x2, y2, color='green')
    # plt.plot(x3, y3, color='green')
    # plt.plot(x4, y4, color='green')
    # plt.plot(x5, y5, color='green')
    # plt.plot(x6, y6, color='green')
    # plt.plot(x7, y7, color='green')
    # plt.plot(x8, y8, color='green')
    # plt.plot(x9, y9, color='green')
    # plt.plot(x10, y10, color='green')
    #
    # plt.plot(x11, y11, color='blue', label='Right-Side Parking')
    # plt.plot(x12, y12, color='blue')
    # plt.plot(x13, y13, color='blue')
    # plt.plot(x14, y14, color='blue')
    # plt.plot(x15, y15, color='blue')
    # plt.plot(x16, y16, color='blue')
    # plt.plot(x17, y17, color='blue')
    # plt.plot(x18, y18, color='blue')
    # plt.plot(x19, y19, color='blue')
    # plt.plot(x20, y20, color='blue')
    #
    # plt.plot(x21, y21, color='magenta', label='Going Straight')
    # plt.plot(x22, y22, color='magenta')
    # plt.plot(x23, y23, color='magenta')
    # plt.plot(x24, y24, color='magenta')
    # plt.plot(x25, y25, color='magenta')
    # plt.plot(x26, y26, color='magenta')
    # plt.plot(x27, y27, color='magenta')
    # plt.plot(x28, y28, color='magenta')
    # plt.plot(x29, y29, color='magenta')
    # plt.plot(x30, y30, color='magenta')
    #
    #
    # plt.legend(loc='lower right')
    # plt.show()


    ############################################################
    ###################### END OF BUTTONS ######################
    ############################################################









    ############################################################
    # helper function to generate random choice based on wieghts
    ############################################################
    def random_action(self):
        actions = [1] * 45 + [2] * 30 + [3] * 25
        return random.choice(actions)


    ###############
    # state mutator
    ###############
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



    ###########################################
    # random action logic inside compute policy
    ###########################################
    def compute_action(self, state):

        print('Compute action ...')

        # TODO: decide the logic to determine the action based on state, right now its random action with weighted options

        print('Generating a random action:')
        action = self.random_action()
        print(action)

        return state, action


    ############################################
    # logic for transition inside compute policy
    ############################################
    def transition(self, state, action):

        print('Transition ...')

        # perform action suggested by compute_action
        req = ActionObservationRequest()
        req.action = action
        res = self.velocity_service1_.call(req)

       # wait for 3 seconds
       # time.sleep(1.0)

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

    ############################
    # function for reward logic
    ############################
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

        print("distance between cars", dist_car)
        print('distance between goal and the car1', dist_goal)
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




    #############################################
    # compute policy loop that computes action,
    # makes the transition and gives reward until
    # goal is achieved
    #############################################
    def policy_loop(self, event):

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
            # current_state = self.update_state()
            # if self.pos_car1[0] >= goal_x and self.pos_car1[1] >= goal_y:
            dist_goal1 = math.sqrt((self.pos_car1[0] - self.car1_goal[0]) ** 2 + (self.pos_car1[1] - self.car1_goal[1]) ** 2)
            dist_goal2 = math.sqrt((self.pos_car2[0] - self.car2_goal[0]) ** 2 + (self.pos_car2[1] - self.car2_goal[1]) ** 2)

            print('dist_goal: ', dist_goal1)
            if dist_goal1 < 2:
                req = ActionObservationRequest()
                req.action = 4
                res = self.velocity_service1_.call(req)
                self.execute_policy=False

            print('dist_goa2: ', dist_goal2)
            if dist_goal2 < 2:
                req = ActionObservationRequest()
                req.action = 4
                res = self.velocity_service2_.call(req)
                self.execute_policy = False



    def save_positions(self, path):
        with open("{}/Trajectory_{}.csv".format(path, time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())), "w") as fp:
            writer = csv.writer(fp, delimiter=",")
            writer.writerow(["time", "x", "y", "z"])
            for point in self.trajectory_collector:
                writer.writerow([point[0], point[1][0], point[1][1], point[1][2]])
                # [ 123123, [1, 2, 3] ]


    def lookup_position(self):
        try:
            translation, rotation = self.listener.lookupTransform("/map", "/car2/base_link", rospy.Time(0))
            self.pos_car2 = translation # [1, 2, 3]
            self.trajectory_collector.append([time.time(), self.pos_car2])
        except (tf.ExtrapolationException, tf.ConnectivityException, tf.LookupException):
            pass

    def trajectory_recording(self, event):
        if self.record_button:
            self.lookup_position()



        # while not rospy.is_shutdown():
        # # while not self.record_button == False:
        #     time_now = rospy.Time(0)
        #     (trans2, rot2) = self.listener.lookupTransform('/map', '/car2/base_link', time_now)
        #     self.pos_car2 = trans2
        #
        #     print ("Current car2 possition: ", self.pos_car2)
        #     # now = rospy.get_rostime()
        #
        #     self.trajectory_collector.append(self.pos_car2)
        #     print('data: ', self.trajectory_collector)
        #     time.sleep(1)





        # time_now = rospy.Time(0)
        # r = rospy.Rate(1)
        ##       (trans1, rot1) = self.listener.lookupTransform('/map', '/car1/base_link', time_now)
        # (trans2, rot2) = self.listener.lookupTransform('/map', '/car2/base_link', time_now)
        ##       # self.pos_car1 = trans1
        # self.pos_car2 = trans2
        # self.trajectory_collector.append(self.pos_car2)
        #
        # print("car 2: ")
        # print(self.pos_car2)
        # print ('data: ', self.trajectory_collector)
