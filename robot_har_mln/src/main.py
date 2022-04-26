#! /usr/bin/env python3

import os
import threading
import copy
from pracmln import MLN, Database, MLNQuery
from pracmln.utils import config, locs
from pracmln.utils.project import PRACMLNConfig
from pracmln.utils.config import global_config_filename
from pracmln.mlnlearn import MLNLearn
import numpy as np
from flask import Flask, request, jsonify
from flask_cors import CORS
import pickle
import yaml

import rospy
import rospkg
import actionlib

from adl_hierarchy_helper import ADLHierarchyHelper
from adl_sequence_modeller import ADLSequenceModeller
from adl_rule_modeller import ADLRuleModeller
from query_selection import QuerySelection
from train_db_consistency_helper import TrainDBConsistencyHelper
from log import Log

from std_msgs import msg
from std_msgs.msg import String

from ronsm_messages.msg import har_simple_evidence
from ronsm_messages.msg import har_reset
from ronsm_messages.msg import har_evidence_list
from ronsm_messages.msg import dm_system_request
from ronsm_messages.msg import har_arm_basic
import ronsm_messages.msg

etypes = ['event', 'percept']

RESET = False

class Main():
    _ros_reason_feedback = ronsm_messages.msg.har_reasonFeedback()
    _ros_reason_result = ronsm_messages.msg.har_reasonResult()
    
    def __init__(self):
        self.id = 'main'
        self.logger = Log(self.id)

        self.logger.startup_msg()

        if RESET:
            self.logger.log_warn('WARNNG: The global reset flag is set to true. This will reset the HAR system and clear the default evidence database.')
            if input('Are you sure you wish to continue? (y/n)') != 'y':
                self.logger.log_warn('Exiting...')
                exit()

        rospy.init_node('robot_har_mln', disable_signals=True)

        # Modes
        self.mode = 'predict' # default is predict

        # Async. Variables
        self.predict_next_cycle = False

        # ROSPack Path
        rospack = rospkg.RosPack()
        self.rel_path = rospack.get_path('robot_har_mln')

        # Helper Classes
        self.qs = QuerySelection()
        self.ahh = ADLHierarchyHelper(self.rel_path)
        self.tdch = TrainDBConsistencyHelper(self.rel_path)
        if RESET:
            self.asm = ADLSequenceModeller(self.rel_path, reset=True)
            self.arm = ADLRuleModeller(self.rel_path, reset=True)
        else:
            self.asm = ADLSequenceModeller(self.rel_path, reset=False)
            self.arm = ADLRuleModeller(self.rel_path, reset=False)

        # Reset Global DB
        if RESET:
            path = self.rel_path + '/src/DBs/global_DB_h.txt'
            try:
                os.remove(path)
            except FileNotFoundError:
                pass
            open(path, 'a').close()

            path = self.rel_path + '/src/DBs/global_DB_s.txt'
            try:
                os.remove(path)
            except FileNotFoundError:
                pass
            open(path, 'a').close()

        # MLN Name
        self.mln_prefix = 'default'

        # Predictions
        self.predictions_h = []
        self.predictions_s = []
        self.predictions_h_f = []
        self.predictions_s_f = []

        # Segmentation
        self.room_e_history = []
        self.room_p_history = []
        self.pred_e_history = []
        self.pred_p_history = []
        self.last_add = None
        self.fork_active = False
        self.predictions_in_segment = 0
        self.predictions_in_segment_f = 0
        self.query_triggered = False
        self.label_save = None
        self.segment_save_e = None
        self.segment_save_p = None

        # Loads
        self.create_pred_store()
        self.load_constants_and_percepts()
        self.create_default_mlns()
        self.init_groundings()
        self.load_global_train_dbs()
        if RESET:
            self.logger.log_warn('WARNING: The MLNs have not been trained. Predictions will be non-sensical until training data has been provided.')
        else:
            self.train_mlns()
            self.save_mlns()

        # ROS Subscribers
        self.sub_sel_evidence = rospy.Subscriber('/ralt_semantic_event_publisher/simple', har_simple_evidence, callback=self.ros_evidence_callback)
        self.sub_ros_evidence = rospy.Subscriber('/robot_har_mln/db/add_delete', har_simple_evidence, callback=self.ros_evidence_callback)
        self.sub_ros_reset = rospy.Subscriber('/robot_har_mln/db/reset', har_reset, callback=self.ros_reset_callback)
        self.sub_ros_add_rule_start = rospy.Subscriber('/robot_har_mln/mln/new_rule_start', String, callback=self.ros_add_rule_start_callback)
        self.sub_ros_add_rule_stop = rospy.Subscriber('/robot_har_mln/mln/new_rule_stop', String, callback=self.ros_add_rule_stop_callback)
        self.sub_ros_add_rule_label = rospy.Subscriber('/robot_har_mln/mln/label', String, callback=self.ros_add_rule_label_callback)
        self.sub_ros_mln_train = rospy.Subscriber('/robot_har_mln/mln/train', String, callback=self.ros_mln_train_callback)
        self.sub_ros_arm_add_rule = rospy.Subscriber('/robot_har_mln/arm/add_rule', har_arm_basic, callback=self.arm.ros_add_rule_callback)

        # ROS Publishers
        self.pub_ros_evidence = rospy.Publisher('/robot_har_mln/db/evidence', har_evidence_list, queue_size=10)
        
        # ROS Action Servers
        self.action_name = 'robot_har_mln/har_reason'
        self._as = actionlib.SimpleActionServer(self.action_name, ronsm_messages.msg.har_reasonAction, execute_cb=self.ros_reason_callback)
        self._as.start()

        self.logger.log_great('Ready.')

    # Init. Methods

    def create_default_mlns(self):
        self.mln_h = MLN(grammar='StandardGrammar', logic='FirstOrderLogic')
        self.mln_s = MLN(grammar='StandardGrammar', logic='FirstOrderLogic')

        self.init_mlns(default_rules=True)

        self.save_mlns()

        self.reset_working_memory()

    def load_global_train_dbs(self):
        path = self.rel_path + '/src/DBs/' + 'global' + '_DB_h.txt'
        self.global_train_db_h = Database.load(self.mln_h, dbfiles=path)

        path = self.rel_path + '/src/DBs/' + 'global' + '_DB_s.txt'
        self.global_train_db_s = Database.load(self.mln_s, dbfiles=path)

    def load_constants_and_percepts(self):
        base_path = self.rel_path + '/src/MLNs/common/'

        event_path = base_path + 'event.txt'
        percept_path = base_path + 'percept.txt'
        predicate_path = base_path + 'predicate.txt'
        rules_h_path = base_path + 'rules_h.txt'
        rules_s_path = base_path + 'rules_s.txt'

        event_file = open(event_path, 'r')
        percept_file = open(percept_path, 'r')
        predicate_file = open(predicate_path, 'r')
        rules_h_file = open(rules_h_path, 'r')
        rules_s_file = open(rules_s_path, 'r')

        self.events = []
        self.events_persist = []
        self.percepts = []
        self.predicates = []
        self.rules_h = []
        self.rules_s = []

        for line in event_file:
            self.events.append(line.rstrip('\n'))
        
        # check if persistent event (starts with !), adds tuple (persistent_event, cancelling_event)
        for i in range(0, len(self.events)):
            if self.events[i][0] == '!':
                self.events[i] = self.events[i].replace('!', '')
                pred_persist = 'involves_event(S,' + self.events[i] + ')'
                pred_cancel = 'involves_event(S,' + self.events[i+1] + ')'
                self.events_persist.append((pred_persist, pred_cancel))

        for line in percept_file:
            self.percepts.append(line.rstrip('\n'))

        for line in predicate_file:
            self.predicates.append(line.rstrip('\n'))

        for line in rules_h_file:
            self.rules_h.append(line.rstrip('\n'))

        for line in rules_s_file:
            self.rules_s.append(line.rstrip('\n'))

    def init_mlns(self, default_rules=False):
        # H
        activity_str = 'activity = {'
        for activity in self.ahh.get_parents():
            activity_str = activity_str + activity + ','
        activity_str = activity_str.rstrip(',') + '}'
        self.mln_h << activity_str
        
        # S
        activity_str = 'activity = {'
        for activity in self.ahh.get_children():
            activity_str = activity_str + activity + ','
        activity_str = activity_str.rstrip(',') + '}'
        self.mln_s << activity_str

        event_str = 'event = {'
        for event in self.events:
            event_str = event_str + event + ','
        event_str = event_str.rstrip(',') + '}'
        self.mln_h << event_str
        self.mln_s << event_str

        percept_str = 'percept = {'
        for percept in self.percepts:
            percept_str = percept_str + percept + ','
        percept_str = percept_str.rstrip(',') + '}'
        self.mln_h << percept_str
        self.mln_s << percept_str

        is_fuzzy = False
        for predicate in self.predicates:
            if is_fuzzy:
                self.mln_h << '#fuzzy \n' + predicate
                self.mln_s << '#fuzzy \n' + predicate
                is_fuzzy = False
            else:
                self.mln_h << predicate
                self.mln_s << predicate
            if predicate == '#fuzzy':
                is_fuzzy = True

        if default_rules:
            for rule in self.rules_h:
                self.mln_h << rule
            for rule in self.rules_s:
                self.mln_s << rule

        self.print_mlns()

    def init_groundings(self):
        self.db_h = Database(self.mln_h)
        self.db_s = Database(self.mln_s)
        
        self.db_h_f = Database(self.mln_h)
        self.db_s_f = Database(self.mln_s)

        for event in self.events:
            predicate = 'involves_event(S,' + event + ')'
            self.db_h << predicate
            self.db_h[predicate] = 0.0
            self.db_s << predicate
            self.db_s[predicate] = 0.0
            self.db_h_f << predicate
            self.db_h_f[predicate] = 0.0
            self.db_s_f << predicate
            self.db_s_f[predicate] = 0.0

        for per in self.percepts:
            predicate = 'involves_percept(S,' + per + ')'
            self.db_h << predicate
            self.db_h[predicate] = 0.0
            self.db_s << predicate
            self.db_s[predicate] = 0.0
            self.db_h_f << predicate
            self.db_h_f[predicate] = 0.0
            self.db_s_f << predicate
            self.db_s_f[predicate] = 0.0

    def create_pred_store(self):
        self.ps = {}
        self.ps['events'] = []
        self.ps['percepts'] = []
        
    # ROS Loop 

    def loop(self):
        while(True):
            while not rospy.core.is_shutdown():
                self.ros_publish()
                waiting_for_label = self.qs.is_waiting_for_label()
                if waiting_for_label:
                    label = self.check_for_label()
                    if label:
                        apply_to_fork = self.qs.apply_to(self.label_save)
                        if apply_to_fork:
                            self.swap_fork_to_main_working_memory()
                    segment = self.check_for_finalised_segment()
                    if label and segment:
                        self.qs.set_waiting_for_response(yes_no=False)
                        self.save_segment()
                if self.predict_next_cycle:
                    self.decay_and_reason() # includes reasoning
                    self.arm.evaluate_rules(self.predictions_h, self.predictions_s, self.room_e_history[-1])
                    self.predict_next_cycle = False
                rospy.rostime.wallsleep(0.5)
            
    # ROS Callbacks

    def ros_evidence_callback(self, msg):
        log = 'Received evidence: (' + msg.etype + ',' + msg.cmd + ',' + msg.evidence + ')'
        self.logger.log(log)
        if msg.cmd == 'add':
            self.add(msg.evidence, msg.etype, msg.room)
        elif msg.cmd == 'delete':
            self.delete(msg.evidence, msg.etype)
        else:
            self.logger.log_warm('Invalid command in ROS evidence topic.')
            
    def ros_reason_callback(self, goal):
        self.logger.log('Received requested to reason.')
        pred, conf = self.reason()
        
        self._result.pred = pred
        self._result.conf = conf
        
        self._as.set_succeeded(self._result)
        
    def ros_reset_callback(self, msg):
        self.logger.log('Received reset command.')
        self.reset_working_memory()

    def ros_add_rule_start_callback(self, msg):
        self.mode = 'train'
        self.asm.start_sequence()
        self.logger.log('Entered training mode.')

    def ros_add_rule_stop_callback(self, msg):
        self.mode = 'predict'
        self.asm.stop_sequence()
        self.logger.log('Entered predict mode.')

    def ros_add_rule_label_callback(self, msg):
        label = msg.data
        if self.mode == 'train':
            log = 'Labelling new rule as: ' + label
            self.logger.log(log)
            self.save_rule(label)
        elif self.mode == 'predict':
            log = 'Received new label for query: ' + label
            self.logger.log(log)
            self.label_save = label
        else:
            log = 'System is invalid mode, unable to handle label callback.'
            self.logger.log_warn(log)

    def ros_mln_train_callback(self, msg):
        self.logger.log('Received request to train MLNs.')
        self.train_mlns()

    # ROS Publishers

    def ros_publish(self):
        msg = har_evidence_list()

        e_preds, e_confs = self.evidence()

        for i in range(0, len(e_confs)):
            e_confs[i] = str(e_confs[i])

        list_str_preds = ''
        list_str_confs = ''
        for i in range(0, len(e_preds)):
            if i < len(e_preds) - 1:
                list_str_preds = list_str_preds + e_preds[i] + ','
                list_str_confs = list_str_confs + e_confs[i] + ','
            else:
                list_str_preds = list_str_preds + e_preds[i]
                list_str_confs = list_str_confs + e_confs[i]

        msg.evidence_preds = list_str_preds
        msg.evidence_confs = list_str_confs

        self.pub_ros_evidence.publish(msg)

    # MLN API Methods

    def print_mlns(self):
        self.logger.log_mini_header('MLN_H')
        self.mln_h.write()
        self.logger.log_mini_header('MLN_S')
        self.mln_s.write()

    def save_mlns(self):
        path = self.rel_path + '/src/MLNs/' + self.mln_prefix + '_MLN_h.mln'
        self.mln_h.tofile(path)

        path = self.rel_path + '/src/MLNs/' + self.mln_prefix + '_MLN_s.mln'
        self.mln_s.tofile(path)

    def save_rule(self, label):
        label_s = label
        label_h = self.ahh.get_parent(label_s)

        self.asm.label_sequence(label_s)

        self.print_mlns()

        self.save_evidence(label_h, label_s)

        self.create_pred_store()

        self.train_mlns()

        self.save_mlns()

    # Deprecated, adds full written rule to MLN instead of using the auto-generated rules
    def save_rule_full(self, label):
        label_s = label
        label_h = self.ahh.get_parent(label_s)

        rule_str = '0.0 '
        for event in self.ps['events']:
            rule_str = rule_str + event
            rule_str = rule_str + ' ^ '
        rule_str = rule_str.rstrip(' ^ ')
        
        rule_str_h = rule_str + ' => ' + 'class(s,' + label_h + ')'
        rule_str_s = rule_str + ' => ' + 'class(s,' + label_s + ')'

        self.mln_h << rule_str_h
        self.mln_s << rule_str_s

        if len(self.ps['percepts']) > 0:
            rule_str = '0.0 '
            for percept in self.ps['percepts']:
                rule_str = rule_str + percept
                rule_str = rule_str + ' ^ '
            rule_str = rule_str.rstrip(' ^ ')

            rule_str_s = rule_str + ' => ' + 'class(s,' + label_s + ')'

            self.mln_s << rule_str_s

        self.asm.label_sequence(label)

        self.print_mlns()

        self.save_mlns()

        self.save_evidence(label_h, label_s)

        self.create_pred_store()

        self.train_mlns()

        self.save_mlns()

    def save_evidence(self, label_h, label_s):
        path_h = self.rel_path + '/src/DBs/' + 'global' + '_DB_h.txt'
        file = open(path_h, 'a')

        for event in self.ps['events']:
            file.write(event + '\n')
        sample_label = 'class(S,' + label_h + ')'
        file.write(sample_label + '\n')
        file.write('---\n')

        file.close()

        events_and_percepts = self.ps['events'] + self.ps['percepts']

        path_s = self.rel_path + '/src/DBs/' + 'global' + '_DB_s.txt'
        file = open(path_s, 'a')

        for event_percept in events_and_percepts:
            file.write(event_percept + '\n')
        sample_label = 'class(S,' + label_s + ')'
        file.write(sample_label + '\n')
        file.write('---\n')

        file.close()

        self.load_global_train_dbs()

    def save_segment(self):
        label = self.label_save
        segment_e = self.segment_save_e
        segment_p = self.segment_save_p

        if segment_p == None:
            segment_p = []

        segment_e = set(segment_e)
        segment_e = list(segment_e)

        segment_p = set(segment_p)
        segment_p = list(segment_p)

        label_s = label
        label_h = self.ahh.get_parent(label_s)

        self.save_evidence_query(segment_e, segment_p, label_h, label_s)

        self.label_save = None
        self.segment_save_e = None
        self.segment_save_p = None

        self.train_mlns()

        self.save_mlns()

    def save_evidence_query(self, segment_e, segment_p, label_h, label_s):
        path_h = self.rel_path + '/src/DBs/' + 'global' + '_DB_h.txt'
        file = open(path_h, 'a')

        for event in segment_e:
            file.write(event + '\n')
        sample_label = 'class(S,' + label_h + ')'
        file.write(sample_label + '\n')
        file.write('---\n')

        file.close()

        events_and_percepts = segment_e + segment_p

        path_s = self.rel_path + '/src/DBs/' + 'global' + '_DB_s.txt'
        file = open(path_s, 'a')

        for event_percept in events_and_percepts:
            file.write(event_percept + '\n')
        sample_label = 'class(S,' + label_s + ')'
        file.write(sample_label + '\n')
        file.write('---\n')

        file.close()

        self.load_global_train_dbs()
    
    def reset_working_memory(self):
        # DBs
        try:
            del self.db_h
            del self.db_s
            del self.ps
        except:
            self.logger.log_warn('Evidence (working memory) databases have not yet be created, they will now be created...')

        self.db_h = Database(self.mln_h)
        self.db_s = Database(self.mln_s)
        self.db_h_f = Database(self.mln_h)
        self.db_s_f = Database(self.mln_s)

        # Predictions
        self.predictions_h = []
        self.predictions_s = []
        self.predictions_h_f = []
        self.predictions_s_f = []

        # Segmentation
        self.room_e_history = []
        self.room_p_history = []
        self.pred_e_history = []
        self.pred_p_history = []
        self.last_add = None

        # Loads
        self.load_constants_and_percepts()
        self.init_groundings()
        self.create_pred_store()
        self.logger.log_great('Evidence (working memory) databases have been (re-)initiatlised.')

    def evidence(self):
        e_preds = []
        e_confs = []

        for e in self.db_s:
            e_preds.append(e[0])
            e_confs.append(e[1])

        return e_preds, e_confs

    def reason(self):
        self.logger.log_mini_header('Evidence State (Main) (H)')
        self.db_h.write()
        self.logger.log_mini_header('Evidence State (Main) (S)')
        self.db_s.write()
        self.reason_h()
        self.reason_s()
        self.predictions_in_segment = self.predictions_in_segment + 1

    def reason_f(self):
        self.logger.log_mini_header('Evidence State (Fork) (H)')
        self.db_h_f.write()
        self.logger.log_mini_header('Evidence State (Fork) (S)')
        self.db_s_f.write()
        self.reason_h_f()
        self.reason_s_f()
        self.predictions_in_segment_f = self.predictions_in_segment_f + 1

    def reason_h(self):
        result = MLNQuery(mln=self.mln_h, db=self.db_h, method='MC-SAT').run() # EnumerationAsk

        queries = []
        for cla in self.ahh.get_parents():
            predicate = 'class(S,' + cla + ')'
            queries.append(predicate)

        probs = []
        for q in queries:
            prob = result.results[q]
            probs.append(prob)
        
        winner = np.argmax(probs)

        parents = self.ahh.get_parents()
        self.logger.log_mini_header('Query Results (H)')
        for i in range(0, len(probs)):
            msg = '(' + parents[i] + ',' + str(probs[i]) + ')'
            self.logger.log(msg)
        msg = 'Prediction (H): (' + parents[winner] + ',' + str(probs[winner]) + ')'
        self.logger.log_great(msg)

        self.predictions_h.append((parents[winner], probs[winner], parents, probs))

    def reason_h_f(self):
        result = MLNQuery(mln=self.mln_h, db=self.db_h_f, method='MC-SAT').run() # EnumerationAsk

        queries = []
        for cla in self.ahh.get_parents():
            predicate = 'class(S,' + cla + ')'
            queries.append(predicate)

        probs = []
        for q in queries:
            prob = result.results[q]
            probs.append(prob)
        
        winner = np.argmax(probs)

        parents = self.ahh.get_parents()
        self.logger.log_mini_header('Query Results (H)')
        for i in range(0, len(probs)):
            msg = '(' + parents[i] + ',' + str(probs[i]) + ')'
            self.logger.log(msg)
        msg = 'Prediction (H): (' + parents[winner] + ',' + str(probs[winner]) + ')'
        self.logger.log_great(msg)

        self.predictions_h_f.append((parents[winner], probs[winner], parents, probs))
    
    def reason_s(self):
        result = MLNQuery(mln=self.mln_s, db=self.db_s, method='MC-SAT').run() # EnumerationAsk

        queries = []
        for cla in self.ahh.get_children(valid_room=self.room_e_history[-1]):
            predicate = 'class(S,' + cla + ')'
            queries.append(predicate)

        probs = []
        for q in queries:
            prob = result.results[q]
            probs.append(prob)
        
        winner = np.argmax(probs)

        children = self.ahh.get_children(valid_room=self.room_e_history[-1])
        self.logger.log_mini_header('Query Results (S)')
        for i in range(0, len(probs)):
            msg = '(' + children[i] + ',' + str(probs[i]) + ')'
            self.logger.log(msg)
        msg = 'Prediction (S): (' + children[winner] + ',' + str(probs[winner]) + ')'
        self.logger.log_great(msg)

        self.predictions_s.append((children[winner], probs[winner], children, probs))

    def reason_s_f(self):
        result = MLNQuery(mln=self.mln_s, db=self.db_s_f, method='MC-SAT').run() # EnumerationAsk

        queries = []
        for cla in self.ahh.get_children(valid_room=self.room_e_history[-1]):
            predicate = 'class(S,' + cla + ')'
            queries.append(predicate)

        probs = []
        for q in queries:
            prob = result.results[q]
            probs.append(prob)
        
        winner = np.argmax(probs)

        children = self.ahh.get_children(valid_room=self.room_e_history[-1])
        self.logger.log_mini_header('Query Results (S)')
        for i in range(0, len(probs)):
            msg = '(' + children[i] + ',' + str(probs[i]) + ')'
            self.logger.log(msg)
        msg = 'Prediction (S): (' + children[winner] + ',' + str(probs[winner]) + ')'
        self.logger.log_great(msg)

        self.predictions_s_f.append((children[winner], probs[winner], children, probs))

    def add(self, evidence, etype, room):
        resp = 'OK. Attempting to add: (' + etype + ',' + evidence + ')'

        if self.valid_pred(evidence, etype):
            self.add_pred(evidence, etype, room)
            self.predict_next_cycle = True
            resp = 'OK. Added: (' + etype + ',' + evidence + ')'
            self.last_add = etype
        else:
            resp = 'Invalid evidence type or invalid predicate. Valid types are: event or percept.'

        self.logger.log(resp)
        return resp

    def add_pred(self, evidence, etype, room):
        if etype == 'event':
            pred = 'involves_event(S,' + evidence + ')'
        elif etype == 'percept':
            pred = 'involves_percept(S,' + evidence + ')'

        if self.mode == 'train':
            self.asm.add_to_sequence(etype, evidence)
            if etype == 'event':
                self.ps['events'].append(pred)
            elif etype == 'percept':
                self.ps['percepts'].append(pred)
        elif self.mode == 'predict':
            if etype == 'event':
                self.db_h[pred] = 1.0
                self.db_s[pred] = 1.0
                if self.fork_active:
                    self.db_h_f[pred] = 1.0
                    self.db_s_f[pred] = 1.0
                self.room_e_history.append(room)
                self.pred_e_history.append(pred)
                canceller, persistent = self.is_canceller(pred)
                if canceller:
                    self.db_h[persistent] = 0.0
                    self.db_s[persistent] = 0.0
                    self.db_h_f[persistent] = 0.0
                    self.db_s_f[persistent] = 0.0
            elif etype == 'percept':
                self.db_s[pred] = 1.0
                self.db_s_f[pred] = 1.0
                self.room_p_history.append(room)
                self.pred_p_history.append(pred)
    
    def valid_pred(self, evidence, etype):
        if etype in etypes:
            if evidence in self.events or evidence in self.percepts:
                return True
            else:
                return False
        else:
            return False

    def delete(self, evidence, etype):
        resp = 'OK'

        if etype  == 'event':
            if evidence in self.events:
                predicate = 'involves_event(S,' + evidence + ')'
                self.db_h[predicate] = 0.0
                self.db_s[predicate] = 0.0
                if self.fork_active:
                    self.db_h_f[predicate] = 0.0
                    self.db_s_f[predicate] = 0.0
            else:
                resp = 'Evidence not modelled in MLN.'
        elif etype == 'percept':
            if evidence in self.percepts:
                predicate = 'involves_percept(S,' + evidence + ')'
                self.db_s[predicate] = 0.0
                if self.fork_active:
                    self.db_s_f[predicate] - 0.0
            else:
                resp = 'Evidence not modelled in MLN.'
        else:
            resp = 'Invalid evidence type. Valid types are: event or percept' 

        self.logger.log(resp)
        return resp

    def train_mlns(self):
        DEFAULT_CONFIG = os.path.join(locs.user_data, global_config_filename)
        conf = PRACMLNConfig(DEFAULT_CONFIG)

        config = {}
        config['verbose'] = True
        config['discr_preds'] = 0
        config['db'] = self.global_train_db_h
        config['mln'] = self.mln_h
        config['ignore_zero_weight_formulas'] = 0
        config['ignore_unknown_preds'] = False
        config['incremental'] = 1
        config['grammar'] = 'StandardGrammar'
        config['logic'] = 'FirstOrderLogic'
        config['method'] = 'pseudo-log-likelihood'
        config['optimizer'] = 'bfgs'
        config['multicore'] = False
        config['profile'] = 0
        config['shuffle'] = 0
        config['prior_mean'] = 0
        config['prior_stddev'] = 5
        config['save'] = True
        config['use_initial_weights'] = 0
        config['use_prior'] = 1

        config['infoInterval'] = 500
        config['resultsInterval'] = 1000
        conf.update(config)

        self.logger.log('Training...')
        learn = MLNLearn(conf, mln=self.mln_h, db=self.global_train_db_h)

        self.mln_h = learn.run()

        # Train S

        DEFAULT_CONFIG = os.path.join(locs.user_data, global_config_filename)
        conf = PRACMLNConfig(DEFAULT_CONFIG)

        config = {}
        config['verbose'] = True
        config['discr_preds'] = 0
        config['db'] = self.global_train_db_s
        config['mln'] = self.mln_s
        config['ignore_zero_weight_formulas'] = 0
        config['ignore_unknown_preds'] = False
        config['incremental'] = 1
        config['grammar'] = 'StandardGrammar'
        config['logic'] = 'FirstOrderLogic'
        config['method'] = 'pseudo-log-likelihood'
        config['optimizer'] = 'bfgs'
        config['multicore'] = False
        config['profile'] = 0
        config['shuffle'] = 0
        config['prior_mean'] = 0
        config['prior_stddev'] = 5
        config['save'] = True
        config['use_initial_weights'] = 0
        config['use_prior'] = 1

        config['infoInterval'] = 500
        config['resultsInterval'] = 1000
        conf.update(config)

        self.logger.log('Training...')
        learn = MLNLearn(conf, mln=self.mln_s, db=self.global_train_db_s)

        self.mln_s = learn.run()

        self.print_mlns()
    
        self.logger.log('Finished training.')

    # Additional Logic

    def decay_and_reason(self):
        clear_e_pred_history = False

        if self.last_add == 'event':
            if len(self.predictions_s) < 1:
                self.logger.log('Cannot decay. No prediction has been made yet.')
            else:
                rooms = self.ahh.get_rooms(self.predictions_s[-1][0])
                if len(rooms[0]) == 0:
                    self.logger.log('Not decaying any predicates, location is consistent.')
        
                elif len(rooms) == 1:
                    if rooms[0] != self.room_e_history[-1]:
                        self.logger.log('Clearing predicate history, location is not consistent.')
                        clear_e_pred_history = True
                    else:
                        self.logger.log('Not decaying any predicates, location is consistent.')

                elif len(rooms) > 1:
                    if self.room_e_history[-1] not in rooms:
                        self.logger.log('Clearing predicate history, location is not consistent.')
                        clear_e_pred_history = True
                    else:
                        self.logger.log('Not decaying any predicates, location is consistent.')

        if clear_e_pred_history:
            self.clear_pred_history_f()
            self.clear_pred_history()
            self.fork_active = False

        self.reason()
        
        if not clear_e_pred_history and not self.fork_active and self.predictions_in_segment > 2:
            consistent_h, consistent_s = self.tdch.is_consistent(self.predictions_h[-2][0], self.predictions_s[-2][0], self.pred_e_history[-1])

            if not consistent_s:
                self.logger.log('Inconsistent event. Now considering that a new activity has occured...')
                self.fork_active = True

                self.db_h_f[self.pred_e_history[-1]] = 1.0
                self.db_s_f[self.pred_e_history[-1]] = 1.0

        consistent_s = None
        consistent_s_f = None
        agree = None
        if self.fork_active:
            self.reason_f()

            if self.predictions_in_segment_f > 1:
                consistent_h, consistent_s = self.tdch.is_consistent(self.predictions_h[-1][0], self.predictions_s[-1][0], self.pred_e_history[-1])
                consistent_h_f, consistent_s_f = self.tdch.is_consistent(self.predictions_h_f[-1][0], self.predictions_s_f[-1][0], self.pred_e_history[-1])
                
                if self.predictions_s[-1][0] == self.predictions_s_f[-1][0]:
                    agree = True
                else:
                    agree = False

                log = 'Consistency checking (consistent_s, consistent_s_f, agree): ' + str(consistent_s) + ', ' + str(consistent_s_f) + ', ' + str(agree)
                self.logger.log(log)

                if not self.query_triggered:
                    self.qs.query_select(self.predictions_h[-1][0], self.predictions_s[-1][0], self.predictions_h_f[-1][0], self.predictions_s_f[-1][0], consistent_s, consistent_s_f, agree)
                    self.segment_save_e = None
                    self.query_triggered = True

    def clear_pred_history(self):
        self.segment_save_e = copy.deepcopy(self.pred_e_history)
        self.segment_save_e.pop()

        new_room = self.room_e_history[-1]
        new_pred = self.pred_e_history[-1]
        for i in range(0, len(self.pred_e_history) - 1):
            if not self.is_persistent(self.pred_e_history[i]):
                self.db_h[self.pred_e_history[i]] = 0.0
                self.db_s[self.pred_e_history[i]] = 0.0
        self.db_h[new_pred] = 1.0
        self.db_s[new_pred] = 1.0
        
        self.room_e_history = []
        self.room_e_history.append(new_room)

        self.pred_e_history = []
        self.pred_e_history.append(new_pred)

        self.predictions_in_segment = 0

    def clear_pred_history_f(self):
        for i in range(0, len(self.pred_e_history)):
            self.db_h_f[self.pred_e_history[i]] = 0.0
            self.db_s_f[self.pred_e_history[i]] = 0.0

        self.predictions_in_segment_f = 0

        self.query_triggered = False

    def is_persistent(self, pred):
        for pair in self.events_persist:
            if pair[0] == pred:
                return True
        return False

    def is_canceller(self, pred):
        for pair in self.events_persist:
            if pair[1] == pred:
                return True, pair[0]
        return False, None

    def swap_fork_to_main_working_memory(self):
        if self.fork_active:
            print('Swapping fork to main')
            self.db_h = copy.deepcopy(self.db_h_f)
            self.db_s = copy.deepcopy(self.db_s_f)

            self.predictions_h = copy.deepcopy(self.predictions_h_f)
            self.predictions_s = copy.deepcopy(self.predictions_s_f)

            print(self.pred_e_history)
            self.pred_e_history = self.pred_e_history[-self.predictions_in_segment_f:]
            print(self.pred_e_history)

            self.predictions_in_segment = copy.deepcopy(self.predictions_in_segment_f)

            self.clear_pred_history_f()
            
            self.fork_active = False

    def check_for_label(self):
        if self.label_save != None:
            return True
        else:
            return False

    def check_for_finalised_segment(self):
        if self.segment_save_e != None:
            return True
        else:
            return False

if __name__ == '__main__':
    m = Main()

    app = Flask(__name__)
    CORS(app)

    threading.Thread(target=lambda: m.loop()).start()

    @app.route('/reset', methods = ['POST'])
    def reset_handler():

        m.reset_working_memory()

        return 'OK'

    @app.route('/evidence', methods = ['POST'])
    def evidence_handler():
        e_preds, e_confs = m.evidence()

        result = {}
        result['e_preds'] = e_preds
        result['e_confs'] = e_confs

        return jsonify(result)

    @app.route('/reason', methods = ['POST'])
    def reason_handler():
        pred, prob = m.reason()

        result = {}
        result['pred'] = pred
        result['prob'] = prob

        return jsonify(result)

    @app.route('/add', methods = ['POST'])
    def add_handler():
        data = request.get_json()

        evidence = data['evidence']
        etype = data['etype']
        room = data['room']

        resp = m.add(evidence, etype, room)

        return resp

    @app.route('/delete', methods = ['POST'])
    def delete_handler():
        data = request.get_json()

        evidence = data['evidence']
        etype = data['etype']

        resp = m.delete(evidence, etype)

        return resp
    
    @app.route('/status', methods = ['GET'])
    def status_handler():
        status = 'OK'

        return jsonify(status)

    @app.route('/')
    def hello_world():
        return '[robot_har_mln] Online.'

    app.run(host='0.0.0.0', port = 5010)
