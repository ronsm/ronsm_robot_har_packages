#!/usr/bin/env python3
from aiohttp import request
import rospy
import rospkg
import sys
from rasa.nlu.model import Interpreter

from dialogue_manager import DialogueManager
from label_linker import LabelLinker
from log import Log
from input_output import InputOutput
from har_interface import HARInterface
from responder import Responder

from ronsm_messages.msg import dm_system_request

har_initiate_adl_query_intents = ['har_adl_label_query']

rasa_primary_model = 'nlu-20220311-132404'

class Main():
    def __init__(self):
        self.id = 'main'
        self.logger = Log(self.id)
        self.logger.startup_msg()

        # ROSPack Path
        rospack = rospkg.RosPack()
        self.rel_path = rospack.get_path('robot_har_dialogue_system')

        self.lock = 0

        self.rospack = rospkg.RosPack()

        self.dataset = 'semantic_ADLs'
        self.label_linker = LabelLinker(self.dataset)
        self.dialogue_manager = DialogueManager(self.rel_path, self.label_linker)

        self.io = InputOutput(self.rel_path)
        self.hi = HARInterface()
        self.responder = Responder(self.rel_path)

        primary_interp_path = self.rospack.get_path('robot_har_dialogue_system') + '/src/rasa/primary/models/' + rasa_primary_model + '/nlu'
        self.primary_interp = Interpreter.load(primary_interp_path)

        rospy.init_node('robot_har_dialogue_system')
        self.sub_system_request = rospy.Subscriber('/robot_har_dialogue_system/system_request', dm_system_request, self.system_request_callback)

        self.logger.log_great('Ready.')

        self.spin()

# Spin

    def spin(self):
        while not rospy.is_shutdown():
            cmd = input('~>')
            if cmd == 'exit':
                sys.exit(0)
            elif cmd == 'x':
                cmd = self.io.listen() # enable this to get input from microphone instead of keyboard
            self.process_user_request(cmd)
            rospy.sleep(0.5)

# Process Requests (System & User)

    def process_system_request(self, intent, args):
        if intent in har_initiate_adl_query_intents:
            self.lock = 1
            self.dialogue_manager.start_query(args)

    def process_user_request(self, text):
        result = self.primary_interp.parse(text)
        intent = result['intent']['name']
        print(result)
        
        if intent == 'start_teaching_adl':
            self.hi.start_teaching_adl()
            self.responder.start_teaching_adl()
        elif intent == 'end_teaching_adl':
            self.responder.stop_teaching_adl()
            label = self.dialogue_manager.story_query_all_labels_teaching()
            if label != '':
                self.hi.label_teaching_adl(label)
            self.hi.stop_teaching_adl() # happens here bc robot_har_mln expects start, label, stop ^\_(*.*)_/^
        elif intent == 'register_marker':
            self.hi.register_marker()
        elif intent == 'look_at_marker':
            self.hi.look_at_marker()
        elif intent == 'create_adl_monitoring_rule':
            self.dialogue_manager.respond_to_input(text)
        else:
            self.logger.log_warn('Invalid intent. Disregarding input.')

        # print(result)

# ROS Callbacks

    def system_request_callback(self, msg):
        self.process_system_request(msg.intent, msg.args)

if __name__ == '__main__':
    m = Main()