#! /usr/bin/env python3

from enum import unique
import pickle
import collections
import pprint
import os
from dataclasses import dataclass
from os.path import exists
from time import perf_counter, sleep
from typing import Optional, List
import copy
from graph_tool.all import *

from regex import D

from adl_hierarchy_helper import ADLHierarchyHelper
from log import Log

@dataclass
class Times:
    global_time: float
    time_since_last: float

@dataclass
class HelpRequest:
    help_requested: bool
    help_types: List[str]

@dataclass
class RobotPose:
    base_x: float
    base_y: float
    base_theta: float
    head_rot: float
    head_height: float

@dataclass
class ChainState:
    uid: Optional[int]
    agent: str
    action: str
    state: List[str]
    time: Times
    robot_pose: Optional[RobotPose]
    help: Optional[HelpRequest]
    count: int

class ADLSequenceModeller():
    def __init__(self, rel_path, reset=False):
        self.id = 'adl_sequence_modeller'
        self.logger = Log(self.id)

        self.rel_path = rel_path
        self.adlhh = ADLHierarchyHelper(self.rel_path)

        self.block_entries = False

        self.pickle_adl_path = self.rel_path + '/src/pickle/ADLs/'
        
        self.status_file = self.pickle_adl_path + 'status.pickle'

        self.markov_chains = {}

        if reset:
            self.logger.log_warn('Reset is set to TRUE. All existing ADL model pickle files will be deleted!')
            decision = input('Are you sure you wish to continue? (y/n) ~> ')
            if decision != 'y':
                self.logger.log_warn('Exiting...')
                exit(0)
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
            obj = self.create_adl_object(adl)
            path = self.pickle_adl_path + adl + '.pickle'
            pickle.dump(obj, open(path, 'wb'))

        self.logger.log_great('Created new ADL model files.')

    def create_adl_object(self, activity):
        obj = {}
        obj['activity'] = activity
        obj['robot_assistance'] = False
        obj['sequences'] = []
        return obj

# Sequence Modeller

    def start_sequence(self):
        self.seq = []
        self.start_time = perf_counter()
        self.prev_time = 0.0
        self.actions = ['START']
        time_object = Times(0.0, 0.0)
        entry = ChainState(None, 'START', 'START', copy.deepcopy(self.actions), time_object, None, None, 1)
        self.seq.append(entry)

    def action(self, agent, action, pose=None):
        if not self.block_entries:
            time = perf_counter() - self.start_time
            elapsed = time - self.prev_time
            time_object = Times("{:.2f}".format(time), "{:.2f}".format(elapsed))
            self.actions.append(action)
            entry = ChainState(None, agent, action, copy.deepcopy(self.actions), time_object, pose, None, 1)
            self.seq.append(entry)
            self.prev_time = time

    def help_request(self, help_type):
        if self.seq[-1].help == None:
            self.seq[-1].help = HelpRequest(True, [help_type])
        else:
            self.seq[-1].help.help_types.append(help_type)

    def stop_sequence(self):
        self.block_entries = True
        time = perf_counter() - self.start_time
        elapsed = time - self.prev_time
        time_object = Times("{:.2f}".format(time), "{:.2f}".format(elapsed))
        self.actions.append('END')
        entry = ChainState(None, 'END', 'END', self.actions, time_object, None, None, 1)
        self.seq.append(entry)

    def label_sequence(self, adl):
        path = self.pickle_adl_path + adl + '.pickle'
        obj = pickle.load(open(path, 'rb'))

        obj['sequences'].append(self.seq)

        pickle.dump(obj, open(path, 'wb'))

        self.block_entries = False

    # Markov Chains

    def update_markov_chain(self, adl):
        path = self.pickle_adl_path + adl + '.pickle'
        obj = pickle.load(open(path, 'rb'))

        unique_chain_states = []

        for seq in obj['sequences']:
            for state in seq:
                if self.is_unique_chain_state(state, unique_chain_states):
                    unique_chain_states.append(state)

        for i in range(0, len(unique_chain_states)):
            unique_chain_states[i].uid = i

        graph = Graph()
        graph.add_vertex(len(unique_chain_states))

        for state in unique_chain_states:
            for state_compare in unique_chain_states:
                if collections.Counter(state_compare.state[:-1]) == collections.Counter(state.state):
                    v1 = graph.vertex(state.uid)
                    v2 = graph.vertex(state_compare.uid)
                    graph.add_edge(v1, v2)

        pdf_file = 'MCs/' + adl + '.pdf'
        graph_draw(graph, vertex_text=graph.vertex_index, output=pdf_file)
        
        msg = 'Saved graph of "' + adl + '" Markov Chain at: ' + pdf_file
        self.logger.log(msg)

        self.logger.log_mini_header('Unique Chain States')
        pprint.pprint(unique_chain_states)

        self.markov_chains[adl] = graph

    def is_unique_chain_state(self, action, unique_chain_states):
        for unique_chain_state in unique_chain_states:
            if collections.Counter(unique_chain_state.state) == collections.Counter(action.state):
                if unique_chain_state.action == action.action:
                    unique_chain_state.count = unique_chain_state.count + 1
                    return False
        return True

if __name__ == '__main__':
    ase = ADLSequenceModeller('/home/ronsm/catkin_ws/src/ronsm_robot_har_packages/robot_har_mln', reset=True)

    # ase.start_sequence()
    # sleep(0.2)
    # ase.action('human', 'Kettle')
    # sleep(0.4)
    # ase.help_request('ExampleHelp')
    # sleep(0.2)
    # pose = RobotPose(1.0, 1.0, 3.14, -0.8, 0.4)
    # ase.action('robot', 'OpenDrawer', pose=pose)
    # sleep(0.3)
    # ase.stop_sequence()
    # ase.label_sequence('Cooking')

    # ase.start_sequence()
    # sleep(0.2)
    # ase.action('human', 'Kettle')
    # sleep(0.4)
    # ase.help_request('ExampleHelp')
    # sleep(0.2)
    # pose = RobotPose(1.0, 1.0, 3.14, -0.8, 0.4)
    # ase.action('robot', 'OpenDrawer', pose=pose)
    # sleep(0.3)
    # ase.stop_sequence()
    # ase.label_sequence('Cooking')

    ase.start_sequence()
    ase.action('human', 'Kettle')
    ase.action('human', 'Tap')
    ase.action('human', 'Bin')
    ase.action('human', 'DrinkwareCabinet')
    ase.action('human', 'CutleryDrawer')
    ase.stop_sequence()
    ase.label_sequence('PreparingDrink')

    ase.start_sequence()
    ase.action('human', 'Bin')
    ase.action('human', 'Tap')
    ase.action('human', 'Kettle')
    ase.action('human', 'DrinkwareCabinet')
    ase.action('human', 'CutleryDrawer')
    ase.stop_sequence()
    ase.label_sequence('PreparingDrink')

    ase.start_sequence()
    ase.action('human', 'Bin')
    ase.action('human', 'Tap')
    ase.action('human', 'Kettle')
    ase.action('human', 'DrinkwareCabinet')
    ase.action('human', 'CutleryDrawer')
    ase.stop_sequence()
    ase.label_sequence('PreparingDrink')

    # ase.start_sequence()
    # ase.action('human', 'Kettle')
    # ase.action('human', 'Tap')
    # ase.action('human', 'Bin')
    # ase.action('human', 'DinnerwareCabinet')
    # ase.action('human', 'DrinkwareCabinet')
    # ase.stop_sequence()
    # ase.label_sequence('PreparingDrink')

    # ase.start_sequence()
    # ase.action('human', 'Kettle')
    # ase.action('human', 'Tap')
    # ase.action('human', 'DrinkwareCabinet')
    # ase.action('human', 'CutleryDrawer')
    # ase.stop_sequence()
    # ase.label_sequence('PreparingDrink')

    # ase.start_sequence()
    # ase.action('human', 'Kettle')
    # ase.action('human', 'Tap')
    # ase.action('human', 'DrinkwareCabinet')
    # ase.action('human', 'CutleryDrawer')
    # ase.stop_sequence()
    # ase.label_sequence('PreparingDrink')

    ase.update_markov_chain('PreparingDrink')

    # ase.start_sequence()
    # ase.action('event', 'event_name')
    # ase.action('action', 'action_name')
    # ase.stop_sequence()
    # ase.label_sequence('Working')

    # ase.start_sequence()
    # ase.action('event', 'event_name')
    # ase.action('action', 'action_name')
    # ase.stop_sequence()
    # ase.label_sequence('Sleeping')