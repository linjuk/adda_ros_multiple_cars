import os
import rospy
import rospkg
import math
import yaml
import roslib

import numpy as np
import time
import actionlib


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
        # make list of actions

        actions = np.arange(10)

        actions[0] = 1
        actions[1] = 1
        actions[2] = 1
        actions[3] = 3
        actions[4] = 3
        actions[5] = 3
        actions[6] = 4

        for i in range(len(actions)):
            print (actions[i])
            req = ActionObservationRequest()
            req.action = actions[i]
            res = self.velocity_service1_.call(req)
            time.sleep(1.0)




        print (len(actions))
        print (' Setup Button pressed, publishing msg')





       
        




       
       

   
     
