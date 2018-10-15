import os
import rospy
import rospkg
import math
import yaml
import roslib

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QEvent, QModelIndex, QObject, Qt, QTimer, Signal, Slot
try:
	from python_qt_binding.QtGui import QShortcut, QTableWidgetItem, QWidget, QLCDNumber, QItemDelegate, QAbstractItemView
except ImportError:
	from python_qt_binding.QtWidgets import QShortcut, QTableWidgetItem, QWidget, QLCDNumber, QItemDelegate, QAbstractItemView

from std_msgs.msg import Bool
from pomdp_car_msgs.srv import ActionObservation, ActionObservationRequest, ActionObservationResponse

class ClassName(QObject):
    _update_task_delegates = Signal()
    _update_endeffector_widget = Signal()
    
    def __init__(self, context):
        QObject.__init__(self, context)
        self.setObjectName('ClassName')

        

        # setup publisher
        # self._examplePublisher = rospy.Publisher('/car_action_gui/exampletopic', Bool,queue_size=10)

       
        # setup services
        self.velocity_service1_ = rospy.ServiceProxy('/car1/car_control/pomdp_velocity', ActionObservation)    
        self.velocity_service2_ = rospy.ServiceProxy('/car2/car_control/pomdp_velocity', ActionObservation)    
        
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
        ui_file = os.path.join(rospkg.RosPack().get_path('car_action_gui'), 'lib', 'ClassName.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ClassNameUi')


        #set connections
        self._widget.Car1_acc_button.pressed.connect(self._on_car1_acc_button_pressed)
        self._widget.Car1_dec_button.pressed.connect(self._on_car1_dec_button_pressed)
        self._widget.Car1_hold_button.pressed.connect(self._on_car1_hold_button_pressed)
        self._widget.Car1_stop_button.pressed.connect(self._on_car1_stop_button_pressed)
        self._widget.Car2_acc_button.pressed.connect(self._on_car2_acc_button_pressed)
        self._widget.Car2_dec_button.pressed.connect(self._on_car2_dec_button_pressed)
        self._widget.Car2_hold_button.pressed.connect(self._on_car2_hold_button_pressed)
        self._widget.Car2_stop_button.pressed.connect(self._on_car2_stop_button_pressed)
        
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
           

   
    def _on_car1_acc_button_pressed(self):
        req = ActionObservationRequest()
        req.action= 3
        res=self.velocity_service1_.call(req)
        print res.current_velocity
 	print 'Button acc car1 msg'

    def _on_car1_dec_button_pressed(self):
        req = ActionObservationRequest()
        req.action= 1
        res=self.velocity_service1_.call(req)
        print res.current_velocity
 	print 'Button dec car1 msg'

    def _on_car1_hold_button_pressed(self):
        req = ActionObservationRequest()
        req.action= 2
        res=self.velocity_service1_.call(req)
        print res.current_velocity
 	print 'Button hold car1 msg'

    def _on_car1_stop_button_pressed(self):
        req = ActionObservationRequest()
        req.action= 4
        res=self.velocity_service1_.call(req)
        print res.current_velocity
 	print 'Button stop car1 msg'

    def _on_car2_acc_button_pressed(self):
        req = ActionObservationRequest()
        req.action= 3
        res=self.velocity_service2_.call(req)
        print res.current_velocity
 	print 'Button acc car2 msg'

    def _on_car2_dec_button_pressed(self):
        req = ActionObservationRequest()
        req.action= 1
        res=self.velocity_service2_.call(req)
        print res.current_velocity
 	print 'Button dec car2 msg'

    def _on_car2_hold_button_pressed(self):
        req = ActionObservationRequest()
        req.action= 2
        res=self.velocity_service2_.call(req)
        print res.current_velocity
 	print 'Button hold car1 msg'

    def _on_car2_stop_button_pressed(self):
        req = ActionObservationRequest()
        req.action= 4
        res=self.velocity_service2_.call(req)
        print res.current_velocity
 	print 'Button stop car2 msg'


       
        




       
       

   
     
