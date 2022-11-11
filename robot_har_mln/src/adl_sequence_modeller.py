#! /usr/bin/env python3

import enum
from operator import index
import pickle
import collections
import pprint
import os
from dataclasses import dataclass
from os.path import exists
from time import perf_counter
from typing import Optional, List, Set
import copy
from lockfile import locked
import numpy as np
from graph_tool.all import *
import networkx as nx
import matplotlib.pyplot as plt
from numpy import False_
from soupsieve import match
import rospy
import threading
from regex import D
import actionlib

from adl_helper import ADLHelper
from log import Log
from object_pool import ObjectPool

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from ronsm_messages.msg import har_percepts, har_percept, dm_intent
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

help_intents = ['intent_pick_up_object', 'intent_go_to_room', 'intent_hand_over', 'intent_align_workspace']
accept_reject_intents = ['intent_accept', 'intent_reject', 'intent_wait']

@dataclass
class Times:
    global_time: float
    time_since_last: float


@dataclass
class HelpRequest:
    help_requested: bool
    help_types: List[List[tuple]]


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
    time: List[Times]
    percepts: Optional[Set[str]]
    manual_alignment: Optional[bool]
    robot_pose: Optional[PoseStamped]
    help: Optional[HelpRequest]
    count: int

@dataclass
class CandidateState:
    adl: str
    state: ChainState
    estimate: str
    probs: Optional[dict]

VERBOSE = True
MAX_WAIT_ACCEPT_REJECT = 20
MAX_WAIT_ACTION_END = 60

class ADLSequenceModeller():
    def __init__(self, rel_path, reset=False):
        self.id = 'adl_sequence_modeller'
        self.logger = Log(self.id)

        self.rel_path = rel_path
        self.adlhh = ADLHelper(self.rel_path, reset=False)
        self.op = ObjectPool()

        self.block_entries = False

        # Mode
        self.mode = 'predict'

        # Pickles
        self.pickle_adl_path = self.rel_path + '/src/pickle/ADLs/'
        self.status_file = self.pickle_adl_path + 'status.pickle'

        # ROS Subscribers
        self.sub_pose = rospy.Subscriber('/global_pose', PoseStamped, callback=self.ros_callback_sub_pose)
        self.current_pose = PoseStamped()
        self.sub_manual_alignment = rospy.Subscriber('/robot_har_help_service/marker_align/aligned', Bool, callback=self.ros_callback_sub_manual_alignment)
        self.sub_help_request = rospy.Subscriber('/robot_har_rasa_core/intent_bus', dm_intent, callback=self.ros_callback_sub_help_request)
        self.sub_action_end = rospy.Subscriber('/robot_har_rasa_core/action_end', String, callback=self.ros_callback_sub_action_end)

        # ROS Publishers
        self.pub_adjust = rospy.Publisher('/robot_har_help_service/marker_align/adjust', PoseStamped, queue_size=10)
        self.pub_offer_help = rospy.Publisher('/robot_har_mln/asm/offer_help', dm_intent, queue_size=10)
        self.pub_intent_bus = rospy.Publisher('/robot_har_rasa_core/intent_bus', dm_intent, queue_size=10)
        self.pub_workspace_pose = rospy.Publisher('/robot_har_help_service/move_to_room/workspace_pose', String, queue_size=10)

        # ROS AS Clients
        self.ros_ac_move_base = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.ros_ac_move_base.wait_for_server()

        # Markov Chains
        self.markov_chains = {}
        self.markov_chains_states = {}

        # State Estimation
        self.state_current_s1 = None
        self.state_current_s2 = None

        self.state_previous_s1 = None
        self.state_previous_s2 = None

        self.state_estimate_success_s1 = False
        self.state_estimate_success_s2 = False

        self.locked = False

        self.accept_reject_help = None
        self.action_end = False
        self.action_success = False

        # Segments (managed by main)
        self.s2_active = False
        self.s1 = []
        self.s2 = []
        self.s1_index = 0
        self.s2_index = 0

        if reset:
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
        adls = self.adlhh.get_adls()

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
            log = 'Invalid mode passed to start_sequence.'
            self.logger.log_warn(log)

    def action(self, mode, agent, action, prediction=None):
        log = 'Action: (' + mode + ', ' + agent + ', ' + action + ', ' + str(prediction) + ')'
        self.logger.log(log)

        if mode == 'train':
            self.save_percepts()
            self.train_action(agent, action)
            self.op.reset()
        elif mode == 'predict':
            self.predict_action(agent, action, prediction)
            self.op.reset()
        else:
            log = 'Invalid mode passed to action.'
            self.logger.log_warn(log)

    def stop_sequence(self, mode):
        self.train_stop_sequence()

    def help_request(self, mode, help_type, help_args):
        self.train_help_request(help_type, help_args)

    def label_sequence(self, mode, adl):
        self.train_label_sequence(adl)

    # Sequence Modeller (Training)

    def train_start_sequence(self):
        self.mode = 'train'
        self.seq = []
        self.start_time = perf_counter()
        self.prev_time = 0.0
        self.actions = ['START']
        time_object = Times(0.0, 0.0)
        entry = ChainState(None, 'START', 'START', copy.deepcopy(
            self.actions), [time_object], None, False, self.current_pose, None, 1)
        self.seq.append(entry)

    def train_action(self, agent, action):
        if not self.block_entries:
            time = perf_counter() - self.start_time
            elapsed = time - self.prev_time
            time_object = Times("{:.2f}".format(
                time), "{:.2f}".format(elapsed))
            self.actions.append(action)
            entry = ChainState(None, agent, action, copy.deepcopy(
                self.actions), [time_object], None, False, self.current_pose, None, 1)
            self.seq.append(entry)
            self.prev_time = time

    def train_help_request(self, help_type, help_args):
        if self.seq[-1].help == None:
            self.seq[-1].help = HelpRequest(True, [[(help_type, help_args)]])
        else:
            self.seq[-1].help.help_types[0].append((help_type, help_args))

    def train_stop_sequence(self):
        if self.mode == 'predict':
            self.seq = copy.deepcopy(self.s1)
        self.block_entries = True
        time = perf_counter() - self.start_time
        elapsed = time - self.prev_time
        time_object = Times("{:.2f}".format(time), "{:.2f}".format(elapsed))
        self.actions.append('END')
        entry = ChainState(None, 'END', 'END', self.actions,
                           [time_object], None, False_, self.current_pose, None, 1)
        self.seq.append(entry)
        self.mode = 'predict'
        self.update_markov_chains()

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
            self.actions), [time_object], None, None, self.current_pose, None, 1)
        self.s1.append(entry)
        self.s1_index = self.s1_index + 1

    def predict_action(self, agent, action, prediction=None):
        time = perf_counter() - self.start_time
        elapsed = time - self.prev_time
        time_object = Times("{:.2f}".format(
            time), "{:.2f}".format(elapsed))
        self.actions.append(action)
        entry = ChainState(self.s1_index, agent, action, copy.deepcopy(
            self.actions), [time_object], None, None, self.current_pose, None, 1)
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
                self.actions), [time_object], None, None, self.current_pose, None, 1)
            self.s2.append(entry)
            self.prev_time = time
            self.s2_index = self.s2_index + 1

        self.predict_step(prediction)

    def predict_step(self, prediction=None):
        if prediction == None:
            self.estimate_state_unbounded()
        else:
            self.estimate_state_bounded(prediction)
        self.next_states()
        self.execute_state()

    def estimate_state_unbounded(self):
        # s1
        print(self.markov_chains_states.items())
        s1_candidates = []
        if len(self.s1) > 0:
            for adl, chain in self.markov_chains_states.items():
                for entry in chain:
                    if collections.Counter(self.s1[-1].state) == collections.Counter(entry.state):
                        if self.s1[-1].state == entry.state:
                            s1_candidates.append(
                                (CandidateState(adl, entry, 'exact', {})))
                        else:
                            s1_candidates.append(
                                (CandidateState(adl, entry, 'approx', {})))

        if len(s1_candidates) > 0:
            if VERBOSE:
                self.logger.log_mini_header('Estimated State (S1) (Unbounded)')
                log = str(s1_candidates)
                self.logger.log(log)
            self.state_estimate_success_s1 = True
            self.select_state(s1_candidates, segment=1)
        else:
            if VERBOSE:
                self.logger.log_mini_header('Estimated State (S1) (Unbounded)')
                log = 'Unable to find any matching candidates.'
                self.logger.log_warn(log)
            self.state_estimate_success_s1 = False

        # s2
        if self.s2_active:
            s2_candidates = []
            if len(self.s2) > 0:
                for adl, chain in self.markov_chains_states.items():
                    for entry in chain:
                        if collections.Counter(self.s2[-1].state) == collections.Counter(entry.state):
                            if self.s2[-1].state == entry.state:
                                s2_candidates.append(
                                    (CandidateState(adl, entry, 'exact', {})))
                            else:
                                s2_candidates.append(
                                    (CandidateState(adl, entry, 'approx', {})))

            if len(s2_candidates) > 0:
                if VERBOSE:
                    self.logger.log_mini_header('Estimated State (S2) (Unbounded)')
                    log = str(s2_candidates)
                    self.logger.log(log)
                self.state_estimate_success_s2 = True
                self.select_state(s2_candidates, segment=2)
            else:
                if VERBOSE:
                    self.logger.log_mini_header('Estimated State (S2) (Unbounded)')
                    log = 'Unable to find any matching candidates.'
                    self.logger.log_warn(log)
                self.state_estimate_success_s2 = False

    def estimate_state_bounded(self, prediction):
        # s1
        s1_candidates = []
        if len(self.s1) > 0:
            chain = self.markov_chains_states[prediction]
            for entry in chain:
                if collections.Counter(self.s1[-1].state) == collections.Counter(entry.state):
                    if self.s1[-1].state == entry.state:
                        s1_candidates.append(
                            (CandidateState(prediction, entry, 'exact', {})))
                    else:
                        s1_candidates.append(
                            (CandidateState(prediction, entry, 'approx', {})))

        if len(s1_candidates) > 0:
            if VERBOSE:
                self.logger.log_mini_header('Estimated State (S1) (Bounded)')
                log = str(s1_candidates)
                self.logger.log(log)
            self.select_state(s1_candidates, segment=1)
            self.state_estimate_success_s1 = True
        else:
            if VERBOSE:
                self.logger.log_mini_header('Estimated State (S1) (Bounded)')
                log = 'Unable to find any matching candidates.'
                self.logger.log_warn(log)
            self.state_estimate_success_s1 = False

        # s2
        if self.s2_active:
            s2_candidates = []
            if len(self.s2) > 0:
                chain = self.markov_chains_states[prediction]
                for entry in chain:
                    if collections.Counter(self.s2[-1].state) == collections.Counter(entry.state):
                        if self.s2[-1].state == entry.state:
                            s2_candidates.append(
                                (CandidateState(prediction, entry, 'exact', {})))
                        else:
                            s2_candidates.append(
                                (CandidateState(prediction, entry, 'approx', {})))

            if len(s2_candidates) > 0:
                if VERBOSE:
                    self.logger.log_mini_header('Estimated State (S2) (Bounded)')
                    log = str(s2_candidates)
                    self.logger.log(log)
                self.select_state(s2_candidates, segment=2)
                self.state_estimate_success_s2 = True
            else:
                if VERBOSE:
                    self.logger.log_mini_header('Estimated State (S2) (Bounded)')
                    log = 'Unable to find any matching candidates.'
                    self.logger.log_warn(log)
                self.state_estimate_success_s2 = False

    def select_state(self, candidates, segment):
        if segment == 1:
            self.state_previous_s1 = copy.deepcopy(self.state_current_s1)
            exact = False
            for candidate in candidates:
                if candidate.estimate == 'exact':
                    self.state_current_s1 = [candidate]
                    exact = True
            if not exact:
                self.state_current_s1 = candidates
        
        if segment == 2:
            self.state_previous_s2 = copy.deepcopy(self.state_current_s2)
            exact = False
            for candidate in candidates:
                if candidate.estimate == 'exact':
                    self.state_current_s2 = [candidate]
                    exact = True
            if not exact:
                self.state_current_s2 = candidates

        if VERBOSE:
            self.logger.log_mini_header('Selected State (S1)')
            log = str(self.state_current_s1)
            self.logger.log(log)

            self.logger.log_mini_header('Selected State (S2)')
            log = str(self.state_current_s2)
            self.logger.log(log)

    def next_states(self):
        # s1
        if self.state_estimate_success_s1:
            if self.state_current_s1[0].estimate == 'exact':
                origin = self.state_current_s1[0].state.uid
                adl = self.state_current_s1[0].adl
                G = self.markov_chains[adl]
                neighbors = G.neighbors(origin)
                for neighbor in neighbors:
                    weight = G.get_edge_data(origin, neighbor)
                    self.state_current_s1[0].probs[neighbor] = weight
            else:
                for candidate in self.state_current_s1:
                    origin = candidate.state.uid
                    adl = candidate.adl
                    G = self.markov_chains[adl]
                    neighbors = G.neighbors(origin)
                    for neighbor in neighbors:
                        weight = G.get_edge_data(origin, neighbor)
                        candidate.probs[neighbor] = weight

            if VERBOSE:
                self.logger.log_mini_header('Next States (S1)')
                log = str(self.state_current_s1)
                self.logger.log(log)

        # s2
        if self.s2_active:
            if self.state_estimate_success_s2:
                if self.state_current_s2[0].estimate == 'exact':
                    origin = self.state_current_s2[0].state.uid
                    adl = self.state_current_s2[0].adl
                    G = self.markov_chains[adl]
                    neighbors = G.neighbors(origin)
                    for neighbor in neighbors:
                        weight = G.get_edge_data(origin, neighbor)
                        self.state_current_s2[0].probs[neighbor] = weight
                else:
                    for candidate in self.state_current_s2:
                        origin = candidate.state.uid
                        adl = candidate.adl
                        G = self.markov_chains[adl]
                        neighbors = G.neighbors(origin)
                        for neighbor in neighbors:
                            weight = G.get_edge_data(origin, neighbor)
                            candidate.probs[neighbor] = weight

                if VERBOSE:
                    self.logger.log_mini_header('Next States (S2)')
                    log = str(self.state_current_s2)
                    self.logger.log(log)

    def execute_state(self):
        x = threading.Thread(target=self.execute_state_thread, args=())
        x.start()
    
    def execute_state_thread(self):
        self.logger.log_mini_header('Execute State(s)')
        
        if self.s2_active:
            if (not self.state_estimate_success_s1) and (not self.state_estimate_success_s2):
                self.locked = False
                return
        else:
            if(not self.state_estimate_success_s1):
                self.locked = False
                return

        self.locked = True

        # what percepts are we expecting?
        expected_percepts = []
        for state in self.state_current_s1:
            if state.state.percepts != None:
                expected_percepts = expected_percepts + state.state.percepts

        if self.s2_active and self.state_estimate_success_s2:
            for state in self.state_current_s2:
                if state.state.percepts != None:
                    expected_percepts = expected_percepts + state.state.percepts
        
        log = 'Expected percepts: ' + str(expected_percepts)
        self.logger.log(log)

        # does the robot need to move?
        if self.state_current_s1[0].estimate == 'exact':
            if self.state_current_s1[0].state.manual_alignment:
                log = 'Robot pose may need to be adjusted.'
                self.logger.log(log)

                msg = dm_intent()
                msg.intent = 'intent_align_workspace'
                msg.args = []
                # self.pub_offer_help.publish(msg)

                # wait = 0
                # while(self.accept_reject_help == None) and (wait < MAX_WAIT_ACCEPT_REJECT):
                #     self.logger.log('Waiting for response...')
                #     wait = wait + 1
                #     rospy.sleep(1)
                # if wait == MAX_WAIT_ACCEPT_REJECT:
                #     log = 'No valid response to help request received in time.'
                #     self.logger.log_warn(log)

                # if self.accept_reject_help == 'intent_accept':
                #     self.adjust_robot_pose(self.state_current_s1[0].state.robot_pose)
                #     log = 'Robot pose is being adjusted.'
                #     self.logger.log(log)
                # else:
                #     return
                self.adjust_robot_pose(self.state_current_s1[0].state.robot_pose)
                log = 'Robot pose is being adjusted.'
                self.logger.log(log)
            else:
                log = 'Robot pose does not need to be adjusted.'
                self.logger.log(log)

        # what help can we offer?
        potential_helps = []
        for state in self.state_current_s1:
            if state.state.help != None:
                potential_helps = potential_helps + state.state.help.help_types

        if self.s2_active and self.state_estimate_success_s2:
            for state in self.state_current_s2:
                if state.state.help != None:
                    potential_helps = potential_helps + state.state.help.help_types

        log = 'Potential help: ' + str(potential_helps)
        self.logger.log(log)            

        locked_path = -1
        for i in range(0, len(potential_helps)):
            print(i)
            item = potential_helps[i][0]

            msg = dm_intent()
            msg.intent = item[0]
            msg.args = item[1]
            self.pub_offer_help.publish(msg)

            wait = 0
            while (self.accept_reject_help == None) and (wait < MAX_WAIT_ACCEPT_REJECT):
                self.logger.log('Waiting for response...')
                wait = wait + 1
                rospy.sleep(1)
            if wait == MAX_WAIT_ACCEPT_REJECT:
                log = 'No valid response to help request received in time.'
                self.logger.log_warn(log)

            if self.accept_reject_help == 'intent_accept':
                locked_path = i
            
            if locked_path != -1:
                break

        self.accept_reject_help = None
        
        if locked_path != -1:
            for i in range(0, len(potential_helps[locked_path])):
                if i == 0:
                    msg = dm_intent()
                    msg.intent = item[0]
                    msg.args = item[1]
                    self.pub_intent_bus.publish(msg)

                    log = 'Waiting for help request action to complete...'
                    self.logger.log(log)
                    wait = 0
                    while (not self.action_end) and (wait < MAX_WAIT_ACTION_END):
                        self.logger.log('Waiting for response...')
                        wait = wait + 1
                        rospy.sleep(1)
                    if wait == MAX_WAIT_ACTION_END:
                        log = 'Action appears not to have completed on time. Help sequence will terminate.'
                        self.logger.log_warn(log)
                        break
                else:
                    msg = dm_intent()
                    msg.intent = potential_helps[locked_path][i][0]
                    msg.args = potential_helps[locked_path][i][1]
                    self.pub_offer_help.publish(msg)

                    wait = 0
                    while (self.accept_reject_help == None) and (wait < MAX_WAIT_ACCEPT_REJECT):
                        self.logger.log('Waiting for response...')
                        wait = wait + 1
                        rospy.sleep(1)
                    if wait == MAX_WAIT_ACCEPT_REJECT:
                        log = 'No valid response to help request received in time.'
                        self.logger.log_warn(log)

                    if self.accept_reject_help == 'intent_accept':
                        msg = dm_intent()
                        msg.intent = item[0]
                        msg.args = item[1]
                        self.pub_intent_bus.publish(msg)

                        log = 'Waiting for help request action to complete...'
                        self.logger.log(log)
                        wait = 0
                        while (not self.action_end) and (wait < MAX_WAIT_ACTION_END):
                            self.logger.log('Waiting for response...')
                            wait = wait + 1
                            rospy.sleep(1)
                        if wait == MAX_WAIT_ACTION_END:
                            log = 'Action appears not to have completed on time. Help sequence will terminate.'
                            self.logger.log_warn(log)
                            break
                        elif self.accept_reject_help == 'intent_reject':
                            break

                    self.accept_reject_help = None
                self.action_end = False
            self.accept_reject_help = None

        # has too much time passed?
        # start a thread that monitors time elapsed and does something
        # when too much time has passed, e.g. +20%

        log = 'State execution thead terminated.'
        self.logger.log(log)

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
                self.actions), [time_object], None, None, self.current_pose, None, 1)
            self.s2.append(entry)
            self.s2_index = self.s2_index + 1

    def save_percepts(self):
        current_percepts = self.op.get_object_pool()
        self.s1[-1].percepts = copy.deepcopy(current_percepts)
        if self.s2_active:
            self.s2[-1].percepts = copy.deepcopy(current_percepts)

    # Markov Chains

    def update_markov_chain(self, adl):
        path = self.pickle_adl_path + adl + '.pickle'
        obj = pickle.load(open(path, 'rb'))

        # create the Markov Chain
        unique_chain_states = []
        if len(obj['sequences']) == 0:
            log = 'Cannot create Markov Chain for ' + adl + ', no sample sequences exist.'
            self.logger.log(log)
            return
        else:
            log = 'Creating Markov Chain for ' + adl + '.'
            self.logger.log(log)

        for seq in obj['sequences']:
            for state in seq:
                is_unique, match_uid = self.is_unique_chain_state(state, unique_chain_states)
                if is_unique:
                    unique_chain_states.append(state)
                else:
                    # help
                    if state.help != None:
                        if unique_chain_states[match_uid].help == None:
                            unique_chain_states[match_uid].help = state.help
                        else:
                            unique_help = True
                            for help_types in unique_chain_states[match_uid].help.help_types:
                                if help_types == state.help.help_types[0]:
                                    unique_help = False
                            if unique_help:
                                unique_chain_states[match_uid].help.help_types.append(state.help.help_types[0])

                    # times
                    unique_chain_states[match_uid].time.append(state.time)

                    # percepts
                    # maximum set?

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

        color_map = []
        for node in G:
            if node == 0:
                color_map.append('green')
            elif G.out_degree(node) == 0:
                color_map.append('red')
            else:
                color_map.append('blue')

        pos = nx.nx_agraph.graphviz_layout(G, prog='neato')
        nx.draw_networkx_nodes(G, pos, node_color=color_map, node_size=800)
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

        png_file = self.rel_path + '/src/MCs/' + adl + '.png'
        plt.savefig(png_file)
        log = 'Updated Markov Chain for "' + adl + \
            '". Saved Markov Chain at: ' + png_file
        self.logger.log(log)

        pprint.pprint(weights)

        self.logger.log_mini_header('Unique Chain States')
        pprint.pprint(unique_chain_states)

        self.markov_chains[adl] = G
        self.markov_chains_states[adl] = unique_chain_states

    def is_unique_chain_state(self, action, unique_chain_states):
        for index, unique_chain_state in enumerate(unique_chain_states):
            if collections.Counter(unique_chain_state.state) == collections.Counter(action.state):
                if unique_chain_state.action == action.action:
                    unique_chain_state.count = unique_chain_state.count + 1
                    return False, index
        return True, 0

    def update_markov_chains(self):
        adls = self.adlhh.get_adls()

        for adl in adls:
            self.update_markov_chain(adl)

    # ROS callbacks

    def ros_callback_sub_pose(self, msg):
        self.current_pose = msg
        if self.mode == 'train':
            self.seq[-1].robot_pose = msg

    def ros_callback_sub_manual_alignment(self, msg):
        self.logger.log('Alignment message received.')
        if msg.data:
            if self.mode == 'train':
                self.seq[-1].manual_alignment = True

    def ros_callback_sub_help_request(self, msg):
        if msg.intent in help_intents and self.mode == 'train':
            log = 'Valid help request message received: ' + msg.intent
            self.logger.log(log)

            x = threading.Thread(target=self.ros_callback_sub_help_request_thread, args=())
            x.start()
        elif msg.intent in accept_reject_intents:
            log = 'Valid accept/reject message received: ' + msg.intent
            self.logger.log(log)

            self.accept_reject_help = msg.intent

    def ros_callback_sub_help_request_thread(self):
        wait = 0
        while (not self.action_end) and (wait < MAX_WAIT_ACTION_END):
            self.logger.log('Waiting for help request to be completed...')
            wait = wait + 1
            rospy.sleep(1)
        if wait == MAX_WAIT_ACTION_END:
            log = 'Action appears not to have completed on time. Help sequence will terminate.'
            self.logger.log_warn(log)

        if self.action_end:
            if self.action_success:
                log = 'Help action completed successfully, action will be added to model.'
                self.logger.log_great(log)
                self.help_request('train', msg.intent, msg.args)
            else:
                log = 'Help action failed, action will not be added to model.'
                self.logger.log_warn(log)

        self.action_end = False
        self.action_success = False

    def ros_callback_sub_action_end(self, msg):
        log = 'Received message confirming help action has ended.'
        self.logger.log(log)
        self.action_end = True
        if msg.data == 'success':
            self.action_success = True
        else:
            self.action_success = False

    # adjust robot pose

    def adjust_robot_pose(self, pose):
        pose.header.stamp = rospy.Time.now()
        
        goal = MoveBaseGoal()
        goal.target_pose = pose

        goal.target_pose.pose.position.x = 0.182
        goal.target_pose.pose.position.y = 1.159

        self.ros_ac_move_base.send_goal(goal)
        
        self.ros_ac_move_base.wait_for_result(rospy.Duration(60))

        action_state = self.ros_ac_move_base.get_state()
        if action_state == GoalStatus.SUCCEEDED:
            self.logger.log_great('Action completed successfully.')
            msg = String()
            self.pub_workspace_pose.publish(msg)
            return True
        else:
            self.logger.log_warn('Action failed to complete. Ensure path to location is not obstructed.')
            return False

if __name__ == '__main__':
    ase = ADLSequenceModeller('/home/ronsm/catkin_ws/src/ronsm_robot_har_packages/robot_har_mln', reset=False)

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

    # ase.start_sequence('train')
    # ase.action('train', 'human', 'Bin')
    # ase.action('train', 'human', 'Tap')
    # ase.action('train', 'human', 'Kettle')
    # ase.action('train', 'human', 'DrinkwareCabinet')
    # ase.action('train', 'human', 'CutleryDrawer')
    # ase.stop_sequence('train')
    # ase.label_sequence('train', 'PreparingDrink')

    # ase.start_sequence('train')
    # ase.action('train', 'human', 'Bin')
    # ase.action('train', 'human', 'Tap')
    # ase.action('train', 'human', 'Kettle')
    # ase.action('train', 'human', 'DrinkwareCabinet')
    # ase.help_request('train', 'ExampleHelp', ['bottle'])
    # ase.action('train', 'human', 'CutleryDrawer')
    # ase.stop_sequence('train')
    # ase.label_sequence('train', 'PreparingDrink')

    # ase.start_sequence('train')
    # ase.action('train', 'human', 'Tap')
    # ase.action('train', 'human', 'Bin')
    # ase.action('train', 'human', 'Kettle')
    # ase.action('train', 'human', 'DrinkwareCabinet')
    # ase.help_request('train', 'ExampleHelp', ['bottle'])
    # ase.help_request('train', 'ExampleHelp2', ['bottle'])
    # ase.action('train', 'human', 'CutleryDrawer')
    # ase.stop_sequence('train')
    # ase.label_sequence('train', 'PreparingDrink')

    ase.start_sequence('predict')
    ase.action('predict', 'human', 'Tap')
    ase.action('predict', 'human', 'Bin')
    ase.action('predict', 'human', 'Kettle')
    ase.action('predict', 'human', 'DrinkwareCabinet')
    # ase.stop_sequence('predict')

    # ase.start_sequence('train')
    # ase.action('train', 'human', 'Bin')
    # ase.action('train', 'human', 'Tap')
    # ase.action('train', 'human', 'Kettle')
    # ase.action('train', 'human', 'CutleryDrawer')
    # ase.action('train', 'human', 'DrinkwareCabinet')
    # ase.stop_sequence('train')
    # ase.label_sequence('train', 'PreparingDrink')

    # ase.start_sequence('train')
    # ase.action('train', 'human', 'Kettle')
    # ase.action('train', 'human', 'FoodCabinet')
    # ase.action('train', 'human', 'MiscCabinet')
    # ase.stop_sequence('train')
    # ase.label_sequence('train', 'Cooking')

    # ase.start_sequence('train')
    # ase.action('train', 'human', 'FoodCabinet')
    # ase.action('train', 'human', 'Kettle')
    # ase.action('train', 'human', 'Fridge')
    # ase.stop_sequence('train')
    # ase.label_sequence('train', 'Cooking')

    # ase.start_sequence('predict')
    # ase.action('predict', 'human', 'FoodCabinet')
    # ase.action('predict', 'human', 'Fridge')
    # ase.action('predict', 'human', 'Bin')
    # ase.stop_sequence('train')
    # ase.label_sequence('train', 'Cooking')

    # ase.start_sequence('predict')
    # ase.action('predict', 'human', 'Bin')
    # ase.action('predict', 'human', 'Tap')
    # ase.action('predict', 'human', 'CutleryDrawer')
    
    # ase.start_sequence('predict')
    # ase.action('predict', 'human', 'Bin')
    # ase.action('predict', 'human', 'Tap')
    # ase.action('predict', 'human', 'CutleryDrawer')

    # ase.start_sequence('predict')
    # ase.action('predict', 'human', 'Bin', prediction='PreparingDrink')
    # ase.action('predict', 'human', 'Tap', prediction='PreparingDrink')
    # ase.action('predict', 'human', 'CutleryDrawer', prediction='PreparingDrink')

    # ase.start_sequence('predict')
    # ase.action('predict', 'human', 'Bin')
    # ase.action('predict', 'human', 'Tap')
    # ase.action('predict', 'human', 'Kettle', prediction='PreparingDrink')

    # ase.start_sequence('predict')
    # ase.action('predict', 'human', 'Kettle')
    # ase.action('predict', 'human', 'FoodCabinet')

    # ase.start_sequence('predict')
    # ase.action('predict', 'human', 'FoodCabinet', prediction='Cooking')
    # ase.action('predict', 'human', 'Fridge', prediction='Cooking')

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
