#!/usr/bin/env python3
import rospy
import rospkg
from rasa.nlu.model import Interpreter

from dialogue_manager import DialogueManager
from label_linker import LabelLinker
from log import Log
from input_output import InputOutput

from ronsm_messages.msg import dm_system_request

har_initiate_adl_query_intents = ["har_initiate_adl_query"]

class Main():
    def __init__(self):
        self.id = 'main'
        self.logger = Log(self.id)

        self.lock = 0

        self.rospack = rospkg.RosPack()

        self.dataset = 'semantic_ADLs'
        self.label_linker = LabelLinker(self.dataset)
        self.dialogue_manager = DialogueManager(self.label_linker)

        self.io = InputOutput()

        primary_interp_path = self.rospack.get_path('robot_har_dialogue_system') + '/src/rasa/primary/models/nlu-20220112-152946/nlu'
        self.primary_interp = Interpreter.load(primary_interp_path)

        rospy.init_node('robot_har_dialogue_system')
        self.sub_system_request = rospy.Subscriber('/robot_har_dialogue_system/system_request', dm_system_request, self.system_request_callback)

        self.logger.log_great('Ready.')

        rospy.spin()

# Process Requests (System & User)

    def process_system_request(self, intent, args):
        if intent in har_initiate_adl_query_intents:
            self.lock = 1
            self.dialogue_manager.start_query(args)

    def process_user_request(self, text):
        result = self.primary_interp.parse(text)
        intent = result['intent']['name']
        print(result['intent']['name'])

        if intent in har_initiate_adl_query_intents:
            self.lock = 1
            self.dialogue_manager.start_query()

# ROS Callbacks

    def system_request_callback(self, msg):
        self.process_system_request(msg.intent, msg.args)

if __name__ == '__main__':
    m = Main()