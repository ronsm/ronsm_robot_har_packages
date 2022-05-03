#! /usr/bin/env python3

import pickle
import pprint
import os
from dataclasses import dataclass
from os.path import exists
from time import perf_counter, sleep
from typing import Optional

from adl_hierarchy_helper import ADLHierarchyHelper
from log import Log

@dataclass
class RobotPose:
    base_x: float
    base_y: float
    base_theta: float
    head_rot: float
    head_height: float

@dataclass
class ADLSequenceEntry:
    etype: str
    entry: str
    global_time: float
    time_since_last: float
    robot_pose: Optional[RobotPose]

class ADLSequenceModeller():
    def __init__(self, rel_path, reset=False):
        self.id = 'adl_sequence_modeller'
        self.logger = Log(self.id)

        self.rel_path = rel_path
        self.adlhh = ADLHierarchyHelper(self.rel_path)

        self.block_entries = False

        self.pickle_adl_path = self.rel_path + '/src/pickle/ADLs/'
        
        self.status_file = self.pickle_adl_path + 'status.pickle'

        if reset:
            os.remove(self.status_file)

        if not exists(self.status_file):
            self.logger.log_warn('ADL model pickle files do not exist. They will be created...')
            self.create_adl_files()
            status = True
            pickle.dump(status, open(self.status_file, 'wb'))

        self.logger.log_great('Ready.')

# File Creation

    def create_adl_files(self):
        adls = self.adlhh.get_children()
        
        for adl in adls:
            obj = self.crete_adl_object(adl)
            path = self.pickle_adl_path + adl + '.pickle'
            pickle.dump(obj, open(path, 'wb'))

        self.logger.log_great('Created new ADL model files.')

    def crete_adl_object(self, name):
        obj = {}
        obj['name'] = name
        obj['robot_assistance'] = False
        obj['sequences'] = []
        return obj

# Sequence Modeller

    def start_sequence(self):
        self.seq = []
        self.start_time = perf_counter()
        self.prev_time = 0.0
        entry = ADLSequenceEntry('start', 'start', 0.00, 0.00, None)
        self.seq.append(entry)

    def add_to_sequence(self, etype, entry, pose=None):
        if not self.block_entries:
            time = perf_counter() - self.start_time
            elapsed = time - self.prev_time
            if etype == 'action':
                entry = ADLSequenceEntry(etype, entry, "{:.2f}".format(time), "{:.2f}".format(elapsed), pose)
            else:
                entry = ADLSequenceEntry(etype, entry, "{:.2f}".format(time), "{:.2f}".format(elapsed), None)
            self.seq.append(entry)
            self.prev_time = time

    def stop_sequence(self):
        self.block_entries = True
        time = perf_counter() - self.start_time
        elapsed = time - self.prev_time
        entry = ADLSequenceEntry('end', 'end', "{:.2f}".format(time), "{:.2f}".format(elapsed), None)
        self.seq.append(entry)

    def label_sequence(self, adl):
        path = self.pickle_adl_path + adl + '.pickle'
        obj = pickle.load(open(path, 'rb'))

        obj['sequences'].append(self.seq)
        pprint.pprint(obj)

        pickle.dump(obj, open(path, 'wb'))

        self.block_entries = False

if __name__ == '__main__':
    ase = ADLSequenceModeller('/home/ronsm/catkin_ws/src/ronsm_robot_har_packages/robot_har_mln', reset=True)

    ase.start_sequence()
    sleep(1.2)
    ase.add_to_sequence('event', 'event_name')
    sleep(2.6)
    pose = RobotPose(1.0, 1.0, 3.14, -0.8, 0.4)
    ase.add_to_sequence('action', 'action_name', pose=pose)
    sleep(1)
    ase.stop_sequence()
    ase.label_sequence('Sleeping')

    ase.start_sequence()
    ase.add_to_sequence('event', 'event_name')
    ase.add_to_sequence('action', 'action_name')
    ase.stop_sequence()
    ase.label_sequence('Working')

    ase.start_sequence()
    ase.add_to_sequence('event', 'event_name')
    ase.add_to_sequence('action', 'action_name')
    ase.stop_sequence()
    ase.label_sequence('Sleeping')