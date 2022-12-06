#!/usr/bin/env python3

# standard libraries
import rospy
from hsrb_interface import Robot, exceptions, collision_world

# internal classes
from log import Log
from speak import Speak
from object_to_tf import ObjectToTF
from pick_from_tf import PickFromTF
from pick_marker import PickMarker
from move_to_room import MoveToRoom
from move_to_pose import MoveToPose
from marker_align import MarkerAlign
from hand_over import HandOver
from collision_array import CollisionArray

# standard messages
from std_msgs.msg import String, Bool

# custom messages
from ronsm_messages.msg import dm_intent

# constants and parameters
# none

class Main():
    def __init__(self):
        # set up logger
        self.id = 'main'
        self.logger = Log(self.id)
        self.logger.startup_msg()

        # set up ROS
        rospy.init_node('robot_har_help_service')

        self.ros_sub_intent_bus = rospy.Subscriber('/robot_har_rasa_core/intent_bus', dm_intent, callback=self.ros_callback_intent_bus)
        self.ros_pub_action_end = rospy.Publisher('/robot_har_rasa_core/action_end', String, queue_size=10)
        self.ros_sub_global_lock = rospy.Subscriber('/ronsm_global_lock', Bool, callback=self.ros_callback_global_lock)

        # set up HSR
        self.robot = Robot()
        try:
            self.logger.log('Waiting on robot resources...')
            self.colw = self.robot.try_get('global_collision_world')
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
        self.pick_marker = PickMarker(self.speak, self.base, self.body, self.grip)
        self.move_to_room = MoveToRoom(self.body)
        self.move_to_pose = MoveToPose(self.body)
        self.marker_align = MarkerAlign(self.speak, self.base, self.body)
        self.hand_over = HandOver(self.speak, self.body, self.grip)
        self.collision_array = CollisionArray(self.colw)

        # set up collision world
        self.body.collision_world = self.colw

        # instance variables
        self.global_lock = False

        # ready       
        self.logger.log_great('Ready.')

        rospy.spin()

    # core logic

    def process_intent(self, intent, args, pose):
        if not self.global_lock:
            log = 'Processing intent: ' + intent
            self.logger.log(log)

            # should match those in robot_har_rasa (mostly)
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
            elif intent == 'intent_move_to_pose':
                if len(args) == 0:
                    self.intent_move_to_pose(target=pose)
                else:
                    self.log_intent_missing_args(intent)
            elif intent == 'intent_hand_over':
                if len(args) == 0:
                    self.intent_hand_over()
                else:
                    self.log_intent_missing_args(intent)
            elif intent == 'intent_register_marker':
                if len(args) == 0:
                    self.marker_align.request_register_marker()
                else:
                    self.log_intent_missing_args(intent)
            elif intent == 'intent_look_at_marker':
                if len(args) == 0:
                    self.marker_align.request_look_at_marker()
                else:
                    self.log_intent_missing_args(intent)
            elif intent == 'intent_test_speech':
                self.speak.request('This is a test of the speech.')
                self.speak.request('This sentence should not interrupt the previous one.')
            else:
                log = 'No service handler available for intent: ' + intent
                self.logger.log_warn(log)
        else:
            log = 'Global lock is active, will not process intents.'
            self.logger.log_warn(log)

    # callbacks

    def ros_callback_intent_bus(self, msg):
        self.process_intent(msg.intent, msg.args, msg.pose)

    def ros_callback_global_lock(self, msg):
        self.global_lock = msg.data

    # help services

    # def intent_pick_up_object(self, target):
    #     success = self.object_to_transform.request(target)
    #     if not success:
    #         say = 'Sorry, I was unable to detect the ' + target
    #         self.speak.request(say)
    #         self.log_action_failure()
    #         return
            
    #     success = self.pick_from_tf.request()
    #     if not success:
    #         say = 'Sorry, I was unable to pick up the ' + target
    #         self.speak.request(say)
    #         self.log_action_failure()
    #         return

    #     say = 'Ok, I have picked up the ' + target 
    #     self.speak.request(say)
    #     self.log_action_success()

    def intent_pick_up_object(self, target):
        success = self.pick_marker.request()
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

    def intent_move_to_pose(self, target):
        success = self.move_to_pose.request(target)
        if not success:
            say = 'Sorry, I was unable to reach that position.'
            self.speak.request(say)
            self.log_action_failure()
            return

        say = 'Ok, I have moved to the next position.'
        self.speak.request(say)
        self.log_action_success()

    def intent_hand_over(self):
        success = self.hand_over.request()
        if not success:
            say = 'Sorry, I was not able to hand over the object.'
            self.speak.request(say)
            self.log_action_failure()
            return
    
        say = 'Ok, I am ready to continue.'
        self.speak.request(say)
        self.log_action_success()

    # common speech

    def log_intent_missing_args(self, intent):
        log = 'Incorrect arguments supplied for intent: ' + intent
        self.logger.log_warn(log)
        say = 'Sorry, I am not sure what you said. Please try again.'
        self.speak.request(say)

    def log_action_success(self):
        self.logger.log_great('Help request actioned successfully.')
        msg = String()
        msg.data = 'success'
        self.ros_pub_action_end.publish(msg)

    def log_action_failure(self):
        self.logger.log_warn('Help request action failure.')
        msg = String()
        msg.data = 'failure'
        self.ros_pub_action_end.publish(msg)

        # intent_pick_up_object
        # obejct = msg.args[0]
        # success = self.ott.request(object)
        # if success: self.pft.request()
        # if success: ok, else not ok

if __name__ == '__main__':
    m = Main()
