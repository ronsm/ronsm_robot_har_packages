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
from global_lock_helper import GlobalLockHelper
from check_preconditions import CheckPreconditions

# standard messages
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

# custom messages
from ronsm_messages.msg import dm_intent, dm_intent_array

# constants and parameters
help_intents = ['intent_pick_up_object', 'intent_go_to_room', 'intent_hand_over', 'intent_align_workspace']
accept_reject_intents = ['intent_accept', 'intent_reject', 'intent_wait']

MAX_WAIT_ACCEPT_REJECT = 20
MAX_WAIT_ACTION_END = 60
MAX_TRIES_ACCEPT_REJECT = 3

class Main():
    def __init__(self):
        # set up logger
        self.id = 'main'
        self.logger = Log(self.id)
        self.logger.startup_msg()

        # set up ROS
        rospy.init_node('robot_har_help_service')

        self.ros_sub_intent_bus = rospy.Subscriber('/robot_har_rasa_core/intent_bus', dm_intent, callback=self.ros_callback_intent_bus)
        self.ros_sub_intent_array = rospy.Subscriber('/robot_har_help_service/intent_array', dm_intent_array, callback=self.ros_callback_intent_array)
        self.ros_sub_global_lock = rospy.Subscriber('/ronsm_global_lock', Bool, callback=self.ros_callback_global_lock)

        self.pub_offer_help = rospy.Publisher('/robot_har_rasa/offer_help', dm_intent, queue_size=10)
        self.ros_pub_action_end = rospy.Publisher('/robot_har_rasa_core/action_end', String, queue_size=10)

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
        self.glh = GlobalLockHelper()
        self.cp = CheckPreconditions()
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
        self.accept_reject_help = None
        self.robot_busy = False

        # ready       
        self.logger.log_great('Ready.')

        rospy.spin()

    # core logic

    def process_intent(self, intent, args, pose):
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
        elif intent == 'intent_open_gripper':
            if len(args) == 0:
                self.intent_open_gripper()
            else:
                self.log_intent_missing_args(intent)
        # elif intent == 'intent_register_marker':
        #     if len(args) == 0:
        #         self.marker_align.request_register_marker()
        #     else:
        #         self.log_intent_missing_args(intent)
        elif intent == 'intent_register_marker':
            if len(args) == 0:
                self.pick_marker.move_to_workspace()
            else:
                self.log_intent_missing_args(intent)
        elif intent == 'intent_look_at_workspace':
            if len(args) == 0:
                self.pick_marker.move_to_workspace()
            else:
                self.log_intent_missing_args(intent)
        elif intent == 'intent_look_at_marker':
            if len(args) == 0:
                self.marker_align.request_look_at_marker()
            else:
                self.log_intent_missing_args(intent)
        else:
            log = 'No service handler available for intent: ' + intent
            self.logger.log_warn(log)

    def process_intent_array(self, msg):
        wait = 0
        while(self.robot_busy):
            self.logger.log('Waiting for robot to be available...')
            if wait == MAX_WAIT_ACTION_END:
                break
            wait = wait + 1
            rospy.sleep(1)

        self.robot_busy = True
        self.logger.log('Processing intents array (help to offer)...')
        for intent in msg.intents:
            affirm = self.execute_wait_for_affirmation(intent.intent, intent.args)
            if affirm:
                self.process_intent(intent.intent, intent.args, intent.pose)
        self.robot_busy = False

    # callbacks

    def ros_callback_intent_bus(self, msg):
        glh_state = self.glh.state()
        if glh_state:
            self.accept_reject_help = msg.intent

        # if self.robot_busy:
        #     self.speak.request('Sorry, I am currently processing your previous request, please try again when I am done.')
        #     return

        self.process_intent(msg.intent, msg.args, msg.pose)

    def ros_callback_intent_array(self, msg):
        self.process_intent_array(msg)

    def ros_callback_global_lock(self, msg):
        self.global_lock = msg.data

    # help services

    def intent_pick_up_object(self, target):
        say = 'Ok, I will try to pick up the ' + target
        self.speak.request(say)
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

    def intent_open_gripper(self):
        success = True
        try:
            say = 'Ok, I will open my gripper in 3... 2... 1... OPENING GRIPPER'
            self.speak.request(say)
            self.grip.set_distance(1.0)
        except:
            success = False

        if not success:
            say = 'Sorry, I was not able to open my gripper.'
            self.speak.request(say)
            self.log_action_failure()
            return

        say = 'Ok, my gripper is open.'
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

    # async waits
    def execute_wait_for_affirmation(self, intent, args):
        affirm = False
        response = False

        self.glh.lock()
        
        tries = 0
        while (not response) and (tries < MAX_TRIES_ACCEPT_REJECT):
            msg = dm_intent()
            msg.intent = intent
            msg.args = args
            msg.pose = PoseStamped()
            self.pub_offer_help.publish(msg)

            wait = 0
            while (self.accept_reject_help == None) and (wait < MAX_WAIT_ACCEPT_REJECT):
                self.logger.log('Waiting for response...')
                wait = wait + 1
                rospy.sleep(1)

            if wait == MAX_WAIT_ACCEPT_REJECT:
                log = 'No valid response to help request received in time.'
                self.logger.log_warn(log)

            if self.accept_reject_help in accept_reject_intents:
                if self.accept_reject_help == 'intent_wait':
                    tries = tries - 1
                    rospy.sleep(10)
                else:
                    response = True

            tries = tries + 1

        if response:
            if self.accept_reject_help == 'intent_accept':
                affirm = True
            else:
                affirm = False

        self.glh.unlock()

        self.accept_reject_help = None
        return affirm

if __name__ == '__main__':
    m = Main()
