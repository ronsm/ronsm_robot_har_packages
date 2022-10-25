#! /usr/bin/env python3

from dataclasses import dataclass
import os
import pickle
from os.path import exists
import rospy

from log import Log
from adl_helper import ADLHelper

from std_msgs.msg import String
from ronsm_messages.msg import har_arm_basic

dos = ['WATCH']

@dataclass
class ADLRuleEntry:
    when: str
    do: str

class ADLRuleModeller():
    def __init__(self, rel_path, reset=False):
        self.id = 'adl_rule_modeller'

        self.logger = Log(self.id)

        self.rel_path = rel_path

        self.pickle_rule_path = self.rel_path + '/src/pickle/rules/rules.pickle'

        self.status_file = self.rel_path + '/src/pickle/rules/status.pickle'

        self.adlhh = ADLHelper(self.rel_path, reset=False)

        self.pub_move_to_room = rospy.Publisher('/robot_har_robot_mover/move_to_room', String, queue_size=10)

        if reset:
            os.remove(self.status_file)

        if not exists(self.status_file):
            self.logger.log_warn('ADL rules pickle file does not exist. It will be created...')
            self.create_rules_file()
            status = True
            pickle.dump(status, open(self.status_file, 'wb'))

        self.load_rules_file()

        self.logger.log_great('Ready.')

    # File Management

    def create_rules_file(self):
        obj = self.create_rules_object()
        pickle.dump(obj, open(self.pickle_rule_path, 'wb'))

        self.logger.log_great('Created new ADL rules file.')

    def create_rules_object(self):
        obj = {}
        obj['sequences'] = []
        return obj

    def load_rules_file(self):
        self.rules_file = pickle.load(open(self.pickle_rule_path, 'rb'))
        print(self.rules_file)

    def save_rules_file(self):
        pickle.dump(self.rules_file, open(self.pickle_rule_path, 'wb'))
        print(self.rules_file)

    # Rule Modeller

    def ros_add_rule_callback(self, msg):
        if self.is_valid(msg.when, msg.do):
            entry = ADLRuleEntry(msg.when, msg.do)
            self.rules_file['sequences'].append(entry)
            self.save_rules_file()
            log = 'Added rule, when: ' + msg.when + ', do: ' + msg.do
            self.logger.log(log)
        else:
            log = 'Invalid rule, when: ' + msg.when + ', do: ' + msg.do
            self.logger.log_warn(log)
    
    def is_valid(self, when, do):
        if (when in self.adlhh.get_adls()) and (do in dos):
            return True
        else:
            return False

    # Pro-Activity

    def evaluate_rules(self, predictions_s, room):
        adl = predictions_s[-1][0]

        first = True
        for entry in self.rules_file['sequences']:
            if entry.when == adl:
                if first:
                    if entry.do == 'WATCH':
                        log = 'Sending robot to room: ' + room + ' to WATCH user while: ' + entry.when
                        self.logger.log(log)
                        msg = String()
                        msg.data = room
                        self.pub_move_to_room.publish(msg)
                        first = False
                    else:
                        log = 'Unimplemented DO for ADL rule: ' + entry.do
                        self.logger.log_warn(log)
                else:
                    self.logger.log_warn('Multiple rules defined for same activity. These are currently unhandled, first rule wile run.')

if __name__ == '__main__':
    arm = ADLRuleModeller()