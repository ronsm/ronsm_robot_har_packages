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

from std_msgs.msg import String
from ronsm_messages.msg import dm_al_request, dm_intent

OUTPUT = 'ROBOT'

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
        self.dialogue_manager = DialogueManager(self.rel_path, self.label_linker, OUTPUT)

        self.io = InputOutput(self.rel_path, OUTPUT)
        self.hi = HARInterface()
        self.responder = Responder(self.rel_path, OUTPUT)

        rospy.init_node('robot_har_dialogue_system')
        self.sub_al_request = rospy.Subscriber('/robot_har_dialogue_system/al_request', dm_al_request, self.callback_al_request)
        self.sub_intent_bus = rospy.Subscriber('/robot_har_rasa_core/intent_bus', dm_intent, self.callback_process_intent)

        self.logger.log_great('Ready.')

        rospy.spin()

    # ROS Callbacks

    def callback_process_intent(self, msg):
        intent = msg.intent
        if intent == 'intent_start_teaching_adl':
            self.hi.start_teaching_adl()
        elif intent == 'intent_end_teaching_adl':
            rospy.sleep(8)
            label = self.dialogue_manager.story_query_all_labels_teaching()
            if label != '':
                self.hi.label_teaching_adl(label)
                rospy.sleep(1.0)
            self.hi.stop_teaching_adl() # happens here bc robot_har_mln expects start, label, stop ^\_(*.*)_/^
    #     elif intent == 'create_adl_monitoring_rule':
    #         self.dialogue_manager.respond_to_input(text)
        else:
            self.logger.log_warn('Invalid intent. Disregarding input.')

    def callback_al_request(self, msg):
        self.lock = 1
        self.dialogue_manager.start_query(msg.args)

if __name__ == '__main__':
    m = Main()