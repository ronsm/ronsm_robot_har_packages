#! /usr/bin/env python3

from enum import unique
import pickle
import collections
import pprint
import os
import math
from dataclasses import dataclass
from os.path import exists
from time import perf_counter, sleep
from turtle import pd
from typing import Optional, List
import copy
import numpy as np
from graph_tool.all import *
import networkx as nx
import matplotlib.pyplot as plt
import rospy
from regex import D

from adl_hierarchy_helper import ADLHierarchyHelper
from log import Log

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from ronsm_messages.msg import har_percepts, har_percept


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


class Percept:
    name: str
    prob: float


@dataclass
class ChainState:
    uid: Optional[int]
    agent: str
    action: str
    state: List[str]
    time: Times
    percepts: Optional[List[Percept]]
    manual_alignment: Optional[bool]
    robot_pose: Optional[Pose2D]
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

        # rospy.init_node('adl_sequence_modeller')
        self.sub_pose = rospy.Subscriber(
            '/robot_har_marker_utility/pose', Pose2D, callback=self.ros_callback_sub_pose)
        self.current_pose = Pose2D()
        self.sub_percept = rospy.Subscriber(
            '/robot_har_percepts/percepts', har_percepts, callback=self.ros_callback_sub_percepts)
        self.current_percepts = {}
        self.sub_manual_alignment = rospy.Subscriber(
            '/robot_har_marker_utility/aligned', Bool, callback=self.ros_callback_sub_manual_alignment)

        self.markov_chains = {}
        self.markov_chains_states = {}

        self.s2_active = False
        self.s1 = []
        self.s2 = []
        self.s1_index = 0
        self.s2_index = 0

        if reset:
            self.logger.log_warn(
                'Reset is set to TRUE. All existing ADL model pickle files will be deleted!')
            decision = input('Are you sure you wish to continue? (y/n) ~> ')
            if decision != 'y':
                self.logger.log_warn('Exiting...')
                exit(0)
            os.remove(self.status_file)

        if not exists(self.status_file):
            self.logger.log_warn(
                'ADL model pickle files do not exist. They will be created...')
            self.create_adl_files()
            status = True
            pickle.dump(status, open(self.status_file, 'wb'))

        self.update_markov_chains()

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

    # Routing

    def start_sequence(self, mode):
        if mode == 'train':
            self.train_start_sequence()
        elif mode == 'predict':
            self.predict_start_sequence()
        else:
            msg = 'Invalid mode passed to start_sequence.'
            self.logger.log_warn(msg)

    def action(self, mode, agent, action):
        self.current_percepts = {}
        if mode == 'train':
            self.train_action(agent, action)
        elif mode == 'predict':
            self.predict_action(agent, action)
        else:
            msg = 'Invalid mode passed to action.'
            self.logger.log_warn(msg)

    def stop_sequence(self, mode):
        self.train_stop_sequence()

    def help_request(self, mode, help_type):
        self.train_help_request(help_type)

    def label_sequence(self, mode, adl):
        self.train_label_sequence(adl)

    # Sequence Modeller (Training)

    def train_start_sequence(self):
        self.seq = []
        self.start_time = perf_counter()
        self.prev_time = 0.0
        self.actions = ['START']
        time_object = Times(0.0, 0.0)
        entry = ChainState(None, 'START', 'START', copy.deepcopy(
            self.actions), time_object, None, None, self.current_pose, None, 1)
        self.seq.append(entry)

    def train_action(self, agent, action):
        if not self.block_entries:
            time = perf_counter() - self.start_time
            elapsed = time - self.prev_time
            time_object = Times("{:.2f}".format(
                time), "{:.2f}".format(elapsed))
            self.actions.append(action)
            entry = ChainState(None, agent, action, copy.deepcopy(
                self.actions), time_object, None, None, self.current_pose, None, 1)
            self.seq.append(entry)
            self.prev_time = time

    def train_help_request(self, help_type):
        if self.seq[-1].help == None:
            self.seq[-1].help = HelpRequest(True, [help_type])
        else:
            self.seq[-1].help.help_types.append(help_type)

    def train_stop_sequence(self):
        self.block_entries = True
        time = perf_counter() - self.start_time
        elapsed = time - self.prev_time
        time_object = Times("{:.2f}".format(time), "{:.2f}".format(elapsed))
        self.actions.append('END')
        entry = ChainState(None, 'END', 'END', self.actions,
                           time_object, None, None, self.current_pose, None, 1)
        self.seq.append(entry)

    def train_label_sequence(self, adl):
        path = self.pickle_adl_path + adl + '.pickle'
        obj = pickle.load(open(path, 'rb'))

        obj['sequences'].append(self.seq)

        pickle.dump(obj, open(path, 'wb'))

        # self.update_markov_chains()

        self.block_entries = False

    # Sequence Predictions (Predict)

    def predict_start_sequence(self):
        self.s1 = []
        self.s1_index = 0
        self.start_time = perf_counter()
        self.prev_time = 0.0
        self.actions = ['START']
        time_object = Times(0.0, 0.0)
        entry = ChainState(self.s1_index, 'START', 'START', copy.deepcopy(
            self.actions), time_object, None, None, self.current_pose, None, 1)
        self.s1.append(entry)
        self.s1_index = self.s1_index + 1

    def predict_action(self, agent, action):
        time = perf_counter() - self.start_time
        elapsed = time - self.prev_time
        time_object = Times("{:.2f}".format(
            time), "{:.2f}".format(elapsed))
        self.actions.append(action)
        entry = ChainState(self.s1_index, agent, action, copy.deepcopy(
            self.actions), time_object, None, None, self.current_pose, None, 1)
        self.s1.append(entry)
        self.prev_time = time
        self.s1_index = self.s1_index + 1

        if self.s2_active:
            time = perf_counter() - self.start_time
            elapsed = time - self.prev_time
            time_object = Times("{:.2f}".format(
                time), "{:.2f}".format(elapsed))
            self.actions.append(action)
            entry = ChainState(self.s2_index, agent, action, copy.deepcopy(
                self.actions), time_object, None, None, self.current_pose, None, 1)
            self.s2.append(entry)
            self.prev_time = time
            self.s2_index = self.s2_index + 1

        self.predict_step()

    def predict_step(self):
        # s1
        s1_candidates = []
        if len(self.s1) > 0:
            for adl, chain in self.markov_chains_states.items():
                for entry in chain:
                    if collections.Counter(self.s1[-1].state) == collections.Counter(entry.state):
                        if self.s1[-1].state == entry.state:
                            s1_candidates.append(
                                (adl, entry.uid, 'exact', entry.state))
                        else:
                            s1_candidates.append(
                                (adl, entry.uid, 'approx', entry.state))

        msg = 'S1 Candidates: ' + str(s1_candidates)
        self.logger.log(msg)

        # s2
        if self.s2_active:
            s2_candidates = []
            if len(self.s2) > 0:
                for adl, chain in self.markov_chains_states.items():
                    for entry in chain:
                        if collections.Counter(self.s2[-1].state) == collections.Counter(entry.state):
                            if self.s2[-1].state == entry.state:
                                s2_candidates.append(
                                    (adl, entry.uid, 'exact', entry.state))
                            else:
                                s2_candidates.append(
                                    (adl, entry.uid, 'approx', entry.state))

            msg = 'S2 Candidates: ' + str(s2_candidates)
            self.logger.log(msg)

    def swap_s2_to_s1(self):
        self.s1 = copy.deepcopy(self.s2)
        self.set_s2_active(False)

    def set_s2_active(self, active):
        self.s2_active = active

        if active:
            self.s2 = []
            self.s2_index = 0
            self.start_time = perf_counter()
            self.prev_time = 0.0
            self.actions = ['START']
            time_object = Times(0.0, 0.0)
            entry = ChainState(self.s2_index, 'START', 'START', copy.deepcopy(
                self.actions), time_object, None, None, self.current_pose, None, 1)
            self.s2.append(entry)
            self.s2_index = self.s2_index + 1

    # Markov Chains

    def update_markov_chain(self, adl):
        path = self.pickle_adl_path + adl + '.pickle'
        obj = pickle.load(open(path, 'rb'))

        # create the Markov Chain
        unique_chain_states = []
        if len(obj['sequences']) == 0:
            msg = 'Cannot create Markov Chain for ' + adl + ', no sample sequences exist.'
            self.logger.log(msg)
            return
        else:
            msg = 'Creating Markov Chain for ' + adl + '.'
            self.logger.log(msg)

        for seq in obj['sequences']:
            for state in seq:
                if self.is_unique_chain_state(state, unique_chain_states):
                    unique_chain_states.append(state)

        for i in range(0, len(unique_chain_states)):
            unique_chain_states[i].uid = i

        adj = np.zeros((len(unique_chain_states), len(
            unique_chain_states)), dtype=np.double)

        state_cnt = 0
        for state in unique_chain_states:
            state_compare_cnt = 0
            for state_compare in unique_chain_states:
                if collections.Counter(state_compare.state[:-1]) == collections.Counter(state.state):
                    adj[state_cnt, state_compare_cnt] = 1
                state_compare_cnt = state_compare_cnt + 1
            state_cnt = state_cnt + 1

        weights = np.zeros((len(unique_chain_states), len(
            unique_chain_states)), dtype=np.double)
        for i in range(0, adj.shape[0]):
            edges_to = []
            for j in range(0, adj.shape[1]):
                if adj[i][j] != 0:
                    edges_to.append(j)
            if len(edges_to) > 0:
                total = 0
                for edge in edges_to:
                    total = total + unique_chain_states[edge].count
                for edge in edges_to:
                    prob = unique_chain_states[edge].count / total
                    weights[i][edge] = prob

        # create and save graph

        G = nx.from_numpy_array(weights, create_using=nx.DiGraph)

        pos = nx.nx_agraph.graphviz_layout(G, prog='neato')
        nx.draw_networkx_nodes(G, pos, node_size=800)
        nx.draw_networkx_edges(G, pos, edgelist=G.edges(),
                               width=1, arrows=True, arrowsize=20)

        nx.draw_networkx_labels(G, pos, font_size=20, font_family="sans-serif")

        edge_labels = dict([((u, v,), f"{d['weight']:.2f}")
                           for u, v, d in G.edges(data=True)])
        nx.draw_networkx_edge_labels(G, pos, edge_labels)

        ax = plt.gca()
        ax.margins(0.08)
        plt.axis("off")
        plt.tight_layout()
        # plt.show(block=False)

        png_file = 'MCs/' + adl + '.png'
        plt.savefig(png_file)
        msg = 'Updated Markov Chain for "' + adl + \
            '". Saved Markov Chain at: ' + png_file
        self.logger.log(msg)

        pprint.pprint(weights)

        self.logger.log_mini_header('Unique Chain States')
        pprint.pprint(unique_chain_states)

        self.markov_chains[adl] = G
        self.markov_chains_states[adl] = unique_chain_states

    def is_unique_chain_state(self, action, unique_chain_states):
        for unique_chain_state in unique_chain_states:
            if collections.Counter(unique_chain_state.state) == collections.Counter(action.state):
                if unique_chain_state.action == action.action:
                    unique_chain_state.count = unique_chain_state.count + 1
                    return False
        return True

    def update_markov_chains(self):
        adls = self.adlhh.get_children()

        for adl in adls:
            self.update_markov_chain(adl)

    # ROS Callbacks
    def ros_callback_sub_pose(self, msg):
        self.current_pose = msg

    def ros_callback_sub_percepts(self, msg):
        for percept in msg.percepts:
            self.current_percepts[percept.name] = percept.conf

        percepts = []
        for percept, conf in self.current_percepts.items():
            p = Percept()
            p.name = percept
            p.conf = conf
            percepts.append(p)

        self.s1[-1].percepts = copy.deepcopy(percepts)
        if self.s2_active:
            self.s2[-1].percepts = copy.deepcopy(percepts)

    def ros_callback_sub_manual_alignment(self, msg):
        if msg.data:
            self.s1[-1].manual_alignment = True
            if self.s2_active:
                self.s2[-1].manual_alignment = True

if __name__ == '__main__':
    ase = ADLSequenceModeller(
        '/home/ronsm/catkin_ws/src/ronsm_robot_har_packages/robot_har_mln', reset=False)

    # ase.start_sequence('train')
    # sleep(0.2)
    # ase.action('train', 'human', 'Kettle')
    # sleep(0.4)
    # ase.train_help_request('ExampleHelp')
    # sleep(0.2)
    # pose = RobotPose(1.0, 1.0, 3.14, -0.8, 0.4)
    # ase.action('train', 'robot', 'OpenDrawer', pose=pose)
    # sleep(0.3)
    # ase.stop_sequence('train')
    # ase.label_sequence('train', 'Cooking')

    # ase.start_sequence('train')
    # sleep(0.2)
    # ase.action('train', 'human', 'Kettle')
    # sleep(0.4)
    # ase.train_help_request('ExampleHelp')
    # sleep(0.2)
    # pose = RobotPose(1.0, 1.0, 3.14, -0.8, 0.4)
    # ase.action('train', 'robot', 'OpenDrawer', pose=pose)
    # sleep(0.3)
    # ase.stop_sequence('train')
    # ase.label_sequence('train', 'Cooking')

    # ase.start_sequence('train')
    # ase.action('train', 'human', 'Kettle')
    # ase.action('train', 'human', 'Tap')
    # ase.action('train', 'human', 'Bin')
    # ase.action('train', 'human', 'DrinkwareCabinet')
    # ase.action('train', 'human', 'CutleryDrawer')
    # ase.stop_sequence('train')
    # ase.label_sequence('train', 'PreparingDrink')

    ase.start_sequence('train')
    ase.action('train', 'human', 'Bin')
    ase.action('train', 'human', 'Tap')
    ase.action('train', 'human', 'Kettle')
    ase.action('train', 'human', 'DrinkwareCabinet')
    ase.action('train', 'human', 'CutleryDrawer')
    ase.stop_sequence('train')
    ase.label_sequence('train', 'PreparingDrink')

    ase.start_sequence('train')
    ase.action('train', 'human', 'Bin')
    ase.action('train', 'human', 'Tap')
    ase.action('train', 'human', 'Kettle')
    ase.action('train', 'human', 'DrinkwareCabinet')
    ase.action('train', 'human', 'CutleryDrawer')
    ase.stop_sequence('train')
    ase.label_sequence('train', 'PreparingDrink')

    ase.start_sequence('train')
    ase.action('train', 'human', 'Tap')
    ase.action('train', 'human', 'Bin')
    ase.action('train', 'human', 'Kettle')
    ase.action('train', 'human', 'DrinkwareCabinet')
    ase.action('train', 'human', 'CutleryDrawer')
    ase.stop_sequence('train')
    ase.label_sequence('train', 'PreparingDrink')

    ase.start_sequence('predict', segment=1)
    ase.action('predict', 'human', 'Bin', segment=1)
    ase.action('predict', 'human', 'Tap', segment=1)

    # ase.start_sequence('train')
    # ase.action('train', 'human', 'Tap')
    # ase.action('train', 'human', 'Bin')
    # ase.action('train', 'human', 'Kettle')
    # ase.action('train', 'human', 'DrinkwareCabinet')
    # ase.action('train', 'human', 'CutleryDrawer')
    # ase.stop_sequence('train')
    # ase.label_sequence('train', 'PreparingDrink')

    # ase.start_sequence('train')
    # ase.action('train', 'human', 'Kettle')
    # ase.action('train', 'human', 'Tap')
    # ase.action('train', 'human', 'Bin')
    # ase.action('train', 'human', 'DinnerwareCabinet')
    # ase.action('train', 'human', 'DrinkwareCabinet')
    # ase.stop_sequence('train')
    # ase.label_sequence('train', 'PreparingDrink')

    # ase.start_sequence('train')
    # ase.action('train', 'human', 'Kettle')
    # ase.action('train', 'human', 'Tap')
    # ase.action('train', 'human', 'DrinkwareCabinet')
    # ase.action('train', 'human', 'CutleryDrawer')
    # ase.stop_sequence('train')
    # ase.label_sequence('train', 'PreparingDrink')

    # ase.start_sequence('train')
    # ase.action('train', 'human', 'Kettle')
    # ase.action('train', 'human', 'Tap')
    # ase.action('train', 'human', 'DrinkwareCabinet')
    # ase.action('train', 'human', 'CutleryDrawer')
    # ase.stop_sequence('train')
    # ase.label_sequence('train', 'PreparingDrink')

    # ase.start_sequence('train')
    # ase.action('train', 'event', 'event_name')
    # ase.action('train', 'action', 'action_name')
    # ase.stop_sequence('train')
    # ase.label_sequence('train', 'Working')

    # ase.start_sequence('train')
    # ase.action('train', 'event', 'event_name')
    # ase.action('train', 'action', 'action_name')
    # ase.stop_sequence('train')
    # ase.label_sequence('train', 'Sleeping')
