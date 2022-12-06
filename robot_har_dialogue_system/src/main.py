#!/usr/bin/env python3
from aiohttp import request
import rospy
import rospkg
import sys

from dialogue_manager import DialogueManager
from label_linker import LabelLinker
from log import Log
from input_output import InputOutput
from har_interface import HARInterface
from responder import Responder

from std_msgs.msg import String
from ronsm_messages.msg import dm_al_request, dm_intent

OUTPUT = 'ROBOT'
MODE = 'STUDY' # NORMAL or STUDY

class Main():
    def __init__(self):
        self.id = 'main'
        self.logger = Log(self.id)
        self.logger.startup_msg()

        # ROSPack Path
        rospack = rospkg.RosPack()
        self.rel_path = rospack.get_path('robot_har_dialogue_system')

        self.lock = 0
        self.global_lock = False

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
        
    def ros_callback_global_lock(self, msg):
        self.global_lock = msg.data

    def callback_process_intent(self, msg):
        if not self.global_lock:
            intent = msg.intent
            if intent == 'intent_start_teaching_adl':
                self.hi.start_teaching_adl()
            elif intent == 'intent_end_teaching_adl':
                if MODE == 'NORMAL':
                    rospy.sleep(8)
                    label = self.dialogue_manager.story_query_all_labels_teaching()
                    if label != '':
                        self.hi.label_teaching_adl(label)
                        rospy.sleep(3.0)
                    else:
                        self.hi.label_teaching_adl('Other')
                        rospy.sleep(3.0)
                    self.hi.stop_teaching_adl() # happens here bc robot_har_mln expects start, label, stop ^\_(*.*)_/^
                elif MODE == 'STUDY':
                    self.hi.label_teaching_adl('Other')
                    rospy.sleep(3)
                    self.hi.stop_teaching_adl() # happens here bc robot_har_mln expects start, label, stop ^\_(*.*)_/^
                else:
                    self.logger.log_warn('Invalid mode specified. Valid modes are NORMAL or ALL_OTHER.')
            else:
                self.logger.log_warn('Invalid intent. Disregarding input.')

    def callback_al_request(self, msg):
        self.lock = 1
        self.dialogue_manager.start_query(msg.args)

if __name__ == '__main__':
    m = Main()
