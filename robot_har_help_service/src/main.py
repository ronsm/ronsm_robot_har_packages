#!/usr/bin/env python3

# standard libraries
from pick_from_tf import PickFromTF
import rospy
from hsrb_interface import Robot, exceptions

# internal classes
from log import Log
from speak import Speak
from object_to_tf import ObjectToTF
from pick_from_tf import PickFromTF
from move_to_room import MoveToRoom

# standard messages
# none

# custom messages
from ronsm_messages.msg import dm_intent

# constants and parameters
# none

class Main():
    def __init__(self):
        # set up logger
        self.id = 'main'
        self.logger = Log(self.id)

        # set up ROS
        rospy.init_node('robot_har_help_service')

        self.ros_sub_intent_bus = rospy.Subscriber('/robot_har_rasa_core/intent_bus', dm_intent, callback=self.ros_callback_intent_bus)

        # set up HSR
        self.robot = Robot()
        try:
            self.logger.log('Waiting on robot resources...')
            self.base = self.robot.try_get('omni_base')
            self.body = self.robot.try_get('whole_body')
            self.grip = self.robot.try_get('gripper')
        except exceptions.ResourceNotFoundError:
            self.logger.log_warn('Unable to get one or more handles for HSR resources. Another process may be using them or the robot is in an error state.')

        # set up classes
        self.speak = Speak()
        self.object_to_transform = ObjectToTF(self.speak)
        self.pick_from_tf = PickFromTF(self.speak, self.base, self.body, self.grip)
        self.move_to_room = MoveToRoom(self.speak)

        # ready
        self.logger.log_great('Ready.')

        rospy.spin()

    def ros_callback_intent_bus(self, msg):
        pass

    def process_intent(self, intent, args):
        pass

        # intent_pick_up_object
        # obejct = msg.args[0]
        # success = self.ott.request(object)
        # if success: self.pft.request()
        # if success: ok, else not ok