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
from marker_align import MarkerAlign

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
            exit(0)

        # set up classes
        self.speak = Speak()
        self.object_to_transform = ObjectToTF(self.speak)
        self.pick_from_tf = PickFromTF(self.speak, self.base, self.body, self.grip)
        self.move_to_room = MoveToRoom(self.speak)
        self.marker_align = MarkerAlign(self.speak, self.base, self.body)

        # ready
        self.speak.request('Ready to help.')
        self.logger.log_great('Ready.')

        rospy.spin()

    # core logic

    def process_intent(self, intent, args):
        log = 'Processing intent: ' + intent
        self.logger.log(log)

        if intent == 'intent_pick_up_object':
            if len(args) == 1:
                self.intent_pick_up_object(target=args[0])
            else:
                self.log_intent_missing_args(intent)
        elif intent == 'intent_go_to_room':
            if len(args) == 1:
                self.intent_go_to_room(target=args[0])
            else:
                self.log_intent_missing_args(intent)
        else:
            log = 'No service handler available for intent: ' + intent
            self.logger.log_warn(log)
            self.speak.request('Sorry, I am unable to help you with that.')

    # callbacks

    def ros_callback_intent_bus(self, msg):
        self.process_intent(msg.intent, msg.args)

    # help services

    def intent_pick_up_object(self, target):
        success = self.object_to_transform.request(target)
        if not success:
            say = 'Sorry, I was unable to detect the ' + target
            self.speak.request(say)
            self.log_action_failure()
            return
            
        success = self.pick_from_tf.request()
        if not success:
            say = 'Sorry, I was unable to pick up the ' + target
            self.speak.request(say)
            self.log_action_failure()
            return

        say = 'Ok, I have picked up the ' + target 
        self.speak.request(say)
        self.log_action_success()

    def intent_go_to_room(self, target):
        success = self.move_to_room.request(target)
        if not success:
            say = 'Sorry, I was unable to get to the ' + target
            self.speak.request(say)
            self.log_action_failure()
            return

        say = 'Ok, I am now in the ' + target
        self.speak.request(say)
        self.log_action_success()

    # common speech

    def log_intent_missing_args(self, intent):
        log = 'Incorrect arguments supplied for intent: ' + intent
        self.logger.log_warn(log)

    def log_action_success(self):
        self.logger.log_great('Help request actioned successfully.')

    def log_action_failure(self):
        self.logger.log_warn('Help request action failure.')

        # intent_pick_up_object
        # obejct = msg.args[0]
        # success = self.ott.request(object)
        # if success: self.pft.request()
        # if success: ok, else not ok

if __name__ == '__main__':
    m = Main()