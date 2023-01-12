#!/usr/bin/env python3

# standard libraries
import sys
import rospy
import rospkg
import requests
from threading import Thread
from queue import Queue

# internal classes
from log import Log
from input_output import InputOutput
from csv_tools import CSVTools

# standard messages
from std_msgs.msg import String

# custom messages
from ronsm_messages.msg import dm_intent

# constants and parameters
RASA_WEBHOOK = 'http://localhost:5005/webhooks/rest/webhook'
RASA_TRACKER = 'http://localhost:5005/conversations/robot_har_rasa/tracker'
RASA_MIN_CONF = 0.75
OUTPUT = 'ROBOT'
INPUT = 'MICROPHONE' # MICROPHONE, KEYBOARD, or REQUEST_ONLY

class Main():
    def __init__(self):
        # set up logger
        self.id = 'main'
        self.logger = Log(self.id)
        self.logger.startup_msg()

        # set up ROS
        rospack = rospkg.RosPack()
        self.rel_path = rospack.get_path('robot_har_rasa')

        self.ros_sub_offer_help = rospy.Subscriber('/robot_har_mln/asm/offer_help', dm_intent, callback=self.ros_callback_offer_help)
        self.ros_sub_text = rospy.Subscriber('/robot_hsr_asr/text', String, callback=self.ros_callback_text)
        self.ros_sub_rasa_utterance_internal = rospy.Subscriber('/robot_har_rasa/rasa_utterance_internal', String, callback=self.ros_callback_rasa_utterance_internal)

        rospy.init_node('robot_har_rasa')

        # set up classes
        self.io = InputOutput(self.rel_path, OUTPUT)
        self.csv_tool = CSVTools()

        # ready
        self.logger.log_great('Ready.')

        if INPUT == 'MICROPHONE':
            self.spin_microphone()
        elif INPUT == 'KEYBOARD':
            self.spin_keyboard()
        elif INPUT == 'REQUEST_ONLY':
            self.spin_request_only()
        else:
            self.logger.log_warn('Invalid input mode specified. Valid options are MICROPHONE, KEYBOARD, or REQUEST_ONLY.')
            sys.exit()

    # spins

    def spin_microphone(self):
        self.io.listen()
        while not rospy.is_shutdown():
            rospy.sleep(1.0)

    def spin_keyboard(self):
        while not rospy.is_shutdown():
            utterance = None
            cmd = input('~>')
            if cmd == 'exit':
                sys.exit(0)
            elif cmd == 'x':
                utterance = input('utterance: ')
            if utterance is not None:
                self.send_to_rasa(utterance)
            rospy.sleep(1)

    def spin_request_only(self):
        while not rospy.is_shutdown():
            rospy.sleep(1)

    # core logic

    def offer_help(self, intent, args):
        # should match those in robot_har_help_service
        if intent == 'intent_pick_up_object':
            say = 'Would you like me to pick up the ' + args[0] + '?'
            self.io.request(say)
        elif intent == 'intent_go_to_room':
            say = 'Shall I go to the ' + args[0] + '?'
            self.io.request(say)
        elif intent == 'intent_align_workspace':
            say = 'Should I move closer to the workspace?'
            self.io.request(say)
        elif intent == 'intent_hand_over':
            say = 'Can I pass you the object?'
            self.io.request(say)
        else:
            log = 'No service handler available for intent: ' + intent
            self.logger.log_warn(log)
            return

        if INPUT == 'KEYBOARD':
            utterance = input('utterance: ')
        elif INPUT == 'MICROPHONE':
            utterance = self.io.listen_once()
        elif INPUT == 'REQUEST_ONLY':
            utterance = self.io.listen_once()

        self.send_to_rasa(utterance)

    # RASA interaction

    def send_to_rasa(self, utterance):
        self.csv_tool.save_utterance(utterance)

        request = {'message': utterance, 'sender': 'robot_har_rasa'}

        response = requests.post(RASA_WEBHOOK, json = request)

        response = response.json()

        valid = requests.get(RASA_TRACKER)

        valid = valid.json()

        log = 'RASA State - Intent: ' + valid['latest_message']['intent']['name'] + ', Confidence: ' + str(valid['latest_message']['intent']['confidence'])
        self.logger.log(log)
        try:
            log = 'RASA Entities - Entity: ' + valid['latest_message']['entities'][0]['entity'] + ', Value: ' + valid['latest_message']['entities'][0]['value']
            self.logger.log(log)
        except:
            self.logger.log_warn('No entities extracted by RASA. This may be expected depending on the intent above.')
            
        try:
            response = response[0]
            message = response['text']
            
            print(message)
            self.io.request(message)
        except:
            self.logger.log_warn('No response provided by RASA. This may be intentional behaviour in some cases.')

    # callbacks

    def ros_callback_offer_help(self, msg):
        self.offer_help(msg.intent, msg.args)

    def ros_callback_text(self, msg):
        self.send_to_rasa(msg.data)

    def ros_callback_rasa_utterance_internal(self, msg):
        self.send_to_rasa(msg.data)

if __name__ == '__main__':
    m = Main()