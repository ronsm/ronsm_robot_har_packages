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

import rospy
import rospkg
import actionlib

from adl_hierarchy_helper import ADLHierarchyHelper
from adl_sequence_modeller import ADLSequenceModeller
from adl_rule_modeller import ADLRuleModeller
from query_selection import QuerySelection
from train_db_consistency_helper import TrainDBConsistencyHelper
from known_domains_helper import KnownDomainsHelper
from log import Log

from std_msgs import msg
from std_msgs.msg import String

from ronsm_messages.msg import har_simple_evidence
from ronsm_messages.msg import har_reset
from ronsm_messages.msg import har_evidence_list
from ronsm_messages.msg import har_arm_basic
from ronsm_messages.msg import har_predictions
import ronsm_messages.msg

etypes = ['event']

RESET = False

MIN_SEGMENT_DEPTH = 2
MIN_SEGMENT_CONF = 0.5

class Main():
    _ros_reason_feedback = ronsm_messages.msg.har_reasonFeedback()
    _ros_reason_result = ronsm_messages.msg.har_reasonResult()
    
    def __init__(self):
        self.id = 'main'
        self.logger = Log(self.id)

        self.logger.startup_msg()

        if RESET:
            self.logger.log_warn('WARNNG: The global reset flag is set to true. This will reset the HAR system and clear the default evidence database.')
            if input('Are you sure you wish to continue? (y/n) ~> ') != 'y':
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
        self.kdh = KnownDomainsHelper(self.rel_path)
        if RESET:
            self.asm = ADLSequenceModeller(self.rel_path, reset=True)
            self.arm = ADLRuleModeller(self.rel_path, reset=True)
        else:
            self.asm = ADLSequenceModeller(self.rel_path, reset=False)
            self.arm = ADLRuleModeller(self.rel_path, reset=False)
        self.asm.start_sequence('predict')

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
        self.s1_predictions_h = []
        self.s1_predictions_s = []
        self.s2_predictions_h = []
        self.s2_predictions_s = []

        # Segmentation
        self.current_room = None
        self.current_evidence = None # used by ASM
        self.room_e_history = []
        self.pred_e_history = []
        self.last_add = None
        self.s2_active = False
        self.s1_predictions_in_segment = 0
        self.s2_predictions_in_segment = 0
        self.query_triggered = False
        self.label_save = None
        self.segment_save_e = None
        self.s1_prediction_h_before_s2 = None
        self.s1_prediction_s_before_s2 = None
        self.s1_consistent_agreement = []
        self.s2_consistent_agreement = []

        # Loads
        self.known_events, self.known_times, self.known_adls = self.kdh.count()
        self.create_pred_store()
        self.load_knowledge_bases()
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
        self.pub_ros_predictions = rospy.Publisher('/robot_har_mln/db/predictions', har_predictions, queue_size=10)
        self.pub_ros_evidence = rospy.Publisher('/robot_har_mln/db/evidence', har_evidence_list, queue_size=10)
        self.pub_ros_move_to_room = rospy.Publisher('/robot_har_robot_mover/move_to_room', String, queue_size=10)
        
        # ROS Action Servers
        self.action_name = 'robot_har_mln/har_reason'
        self._as = actionlib.SimpleActionServer(self.action_name, ronsm_messages.msg.har_reasonAction, execute_cb=self.ros_reason_callback)
        self._as.start()

        self.logger.log_great('Ready.')

    # Init. Methods

    def create_default_mlns(self):
        self.mln_h = MLN(grammar='PRACGrammar', logic='FirstOrderLogic')
        self.mln_s = MLN(grammar='PRACGrammar', logic='FirstOrderLogic')

        self.init_mlns(default_rules=True)

        self.save_mlns()

        self.reset_working_memory()

    def load_global_train_dbs(self):
        path = self.rel_path + '/src/DBs/' + 'global' + '_DB_h.txt'
        self.global_train_db_h = Database.load(self.mln_h, dbfiles=path)

        path = self.rel_path + '/src/DBs/' + 'global' + '_DB_s.txt'
        self.global_train_db_s = Database.load(self.mln_s, dbfiles=path)

    def load_knowledge_bases(self):
        base_path = self.rel_path + '/src/MLNs/common/'

        event_path = base_path + 'event.txt'
        time_path = base_path + 'time.txt'
        predicate_path = base_path + 'predicate.txt'
        rules_h_path = base_path + 'rules_h.txt'
        rules_s_path = base_path + 'rules_s.txt'

        event_file = open(event_path, 'r')
        time_file = open(time_path, 'r')
        predicate_file = open(predicate_path, 'r')
        rules_h_file = open(rules_h_path, 'r')
        rules_s_file = open(rules_s_path, 'r')

        self.events = []
        self.times = []
        self.events_persist = []
        self.predicates = []
        self.rules_h = []
        self.rules_s = []

        for line in event_file:
            self.events.append(line.rstrip('\n'))
        
        # check if persistent event (starts with !), adds tuple (persistent_event, cancelling_event)
        for i in range(0, len(self.events)):
            if self.events[i][0] == '!':
                self.events[i] = self.events[i].replace('!', '')
                pred_persist = 'involves(S,' + self.events[i] + ')'
                pred_cancel = 'involves(S,' + self.events[i+1] + ')'
                self.events_persist.append((pred_persist, pred_cancel))

        for line in time_file:
            self.times.append(line.rstrip('\n'))

        for line in predicate_file:
            self.predicates.append(line.rstrip('\n'))

        for line in rules_h_file:
            self.rules_h.append(line.rstrip('\n'))

        for line in rules_s_file:
            self.rules_s.append(line.rstrip('\n'))

    def init_mlns(self, default_rules=False):
        # # H Only
        # activity_str = 'activity = {'
        # for activity in self.ahh.get_parents():
        #     activity_str = activity_str + activity + ','
        # activity_str = activity_str.rstrip(',') + '}'
        # self.mln_h << activity_str
        
        # # S Only
        # activity_str = 'activity = {'
        # for activity in self.ahh.get_children():
        #     activity_str = activity_str + activity + ','
        # activity_str = activity_str.rstrip(',') + '}'
        # self.mln_s << activity_str

        # Both
        # event_str = 'event = {'
        # for event in self.events:
        #     event_str = event_str + event + ','
        # event_str = event_str.rstrip(',') + '}'
        # self.mln_h << event_str
        # self.mln_s << event_str

        # time_str = 'time = {'
        # for time in self.times:
        #     time_str = time_str + time + ','
        # time_str = time_str.rstrip(',') + '}'
        # self.mln_h << time_str
        # self.mln_s << time_str

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
        self.s1_db_h = Database(self.mln_h)
        self.s1_db_s = Database(self.mln_s)
        
        self.s2_db_h = Database(self.mln_h)
        self.s2_db_s = Database(self.mln_s)

        for event in self.known_events:
            predicate = 'involves(S,' + event + ')'
            self.s1_db_h << predicate
            self.s1_db_h[predicate] = 0.0
            self.s1_db_s << predicate
            self.s1_db_s[predicate] = 0.0
            self.s2_db_h << predicate
            self.s2_db_h[predicate] = 0.0
            self.s2_db_s << predicate
            self.s2_db_s[predicate] = 0.0

        for time in self.known_times:
            predicate = 'occurs(S, ' + time + ')'
            self.s1_db_h << predicate
            self.s1_db_h[predicate] = 0.0
            self.s1_db_s << predicate
            self.s1_db_s[predicate] = 0.0
            self.s2_db_h << predicate
            self.s2_db_h[predicate] = 0.0
            self.s2_db_s << predicate
            self.s2_db_s[predicate] = 0.0

        for event_o in self.known_events:
            for event_i in self.known_events:
                predicate = 'after(' + event_o + ',' + event_i + ')'
                self.s1_db_h << predicate
                self.s1_db_h[predicate] = 0.0
                self.s1_db_s << predicate
                self.s1_db_s[predicate] = 0.0
                self.s2_db_h << predicate
                self.s2_db_h[predicate] = 0.0
                self.s2_db_s << predicate
                self.s2_db_s[predicate] = 0.0

    def create_pred_store(self):
        self.ps = {}
        self.ps['events'] = []
        
    # ROS Loop 

    def loop(self):
        while(True):
            while not rospy.core.is_shutdown():
                self.ros_pub_evidence()
                waiting_for_label = self.qs.is_waiting_for_label()
                if waiting_for_label:
                    label = self.check_for_label()
                    if label:
                        apply_to_s2 = self.qs.apply_to(self.label_save)
                        if apply_to_s2:
                            self.swap_s2_to_s1()
                    segment = self.check_for_finalised_segment()
                    if label and segment:
                        self.qs.set_waiting_for_response(yes_no=False)
                        self.save_segment()
                if self.predict_next_cycle:
                    self.decay_and_reason() # includes reasoning
                    prediction_h, prediction_s = self.get_prevailing_predictions()
                    self.asm.action('predict', 'human', self.current_evidence, prediction=prediction_s)
                    self.ros_pub_predictions(prediction_h, prediction_s)
                    self.arm.evaluate_rules(self.s1_predictions_h, self.s1_predictions_s, self.room_e_history[-1])
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
        pred, conf = self.s1_reason()
        
        self._result.pred = pred
        self._result.conf = conf
        
        self._as.set_succeeded(self._result)
        
    def ros_reset_callback(self, msg):
        self.logger.log('Received reset command.')
        self.reset_working_memory()

    def ros_add_rule_start_callback(self, msg):
        self.mode = 'train'
        self.asm.start_sequence('train')
        self.logger.log('Entered training mode.')

    def ros_add_rule_stop_callback(self, msg):
        self.mode = 'predict'
        self.logger.log('Entered predict mode.')

    def ros_add_rule_label_callback(self, msg):
        label = msg.data
        if label == 'LABEL_ERROR':
            log = 'Labelling error. Invalid label received from dialogue system. Query will be cancelled and segment lost.'
            self.logger.log_warn(log)
            self.qs.cancel_query()

        if self.mode == 'train':
            log = 'Labelling new rule as: ' + label
            self.logger.log(log)
            self.save_rule(label)
        elif self.mode == 'predict':
            log = 'Received new label for query: ' + label
            self.logger.log(log)
            self.label_save = label
        else:
            log = 'System is in invalid mode, unable to handle label callback.'
            self.logger.log_warn(log)

    def ros_add_rule_label_callback_internal(self, label):
        if self.mode == 'train':
            log = 'Labelling new rule as: ' + label
            self.logger.log(log)
            self.save_rule(label)
        elif self.mode == 'predict':
            log = 'Received new label for query: ' + label
            self.logger.log(log)
            self.label_save = label
        else:
            log = 'System is in invalid mode, unable to handle label callback.'
            self.logger.log_warn(log)

    def ros_mln_train_callback(self, msg):
        self.logger.log('Received request to train MLNs.')
        self.train_mlns()

    def ros_move_to_room(self, room):
        if (room != self.current_room) or (self.current_room == None):
            log = 'Sending robot to room: ' + room
            self.logger.log(log)
            msg = String()
            msg.data = room
            self.pub_ros_move_to_room.publish(msg)
            self.current_room = room

    # ROS Publishers

    def ros_pub_predictions(self, prediction_h, prediction_s):
        msg = har_predictions()
        msg.h = prediction_h
        msg.s = prediction_s
        self.pub_ros_predictions.publish(msg)

    def ros_pub_evidence(self):
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

        self.asm.stop_sequence('train')
        self.asm.label_sequence('train', label_s)

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

        self.asm.label_sequence('train', label_s)

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

        path_s = self.rel_path + '/src/DBs/' + 'global' + '_DB_s.txt'
        file = open(path_s, 'a')

        for event in self.ps['events']:
            file.write(event + '\n')
        sample_label = 'class(S,' + label_s + ')'
        file.write(sample_label + '\n')
        file.write('---\n')

        file.close()

        self.load_global_train_dbs()

    def save_segment(self):
        self.asm.stop_sequence('train')
        self.asm.label_sequence('train', label_s)

        label = self.label_save
        segment_e = self.segment_save_e

        segment_e = set(segment_e)
        segment_e = list(segment_e)

        label_s = label
        label_h = self.ahh.get_parent(label_s)

        self.save_evidence_query(segment_e, label_h, label_s)

        self.label_save = None
        self.segment_save_e = None

        self.train_mlns()

        self.save_mlns()

    def save_evidence_query(self, segment_e, label_h, label_s):
        path_h = self.rel_path + '/src/DBs/' + 'global' + '_DB_h.txt'
        file = open(path_h, 'a')

        for event in segment_e:
            file.write(event + '\n')
        sample_label = 'class(S,' + label_h + ')'
        file.write(sample_label + '\n')
        file.write('---\n')

        file.close()

        path_s = self.rel_path + '/src/DBs/' + 'global' + '_DB_s.txt'
        file = open(path_s, 'a')

        for event in segment_e:
            file.write(event + '\n')
        sample_label = 'class(S,' + label_s + ')'
        file.write(sample_label + '\n')
        file.write('---\n')

        file.close()

        self.load_global_train_dbs()
    
    def reset_working_memory(self):
        # DBs
        try:
            del self.s1_db_h
            del self.s1_db_s
            del self.ps
        except:
            self.logger.log_warn('Evidence (working memory) databases have not yet be created, they will now be created...')

        self.s1_db_h = Database(self.mln_h)
        self.s1_db_s = Database(self.mln_s)
        self.s2_db_h = Database(self.mln_h)
        self.s2_db_s = Database(self.mln_s)

        # Predictions
        self.s1_predictions_h = []
        self.s1_predictions_s = []
        self.s2_predictions_h = []
        self.s2_predictions_s = []

        # Segmentation
        self.room_e_history = []
        self.pred_e_history = []
        self.last_add = None

        # Loads
        self.load_knowledge_bases()
        self.init_groundings()
        self.create_pred_store()
        self.logger.log_great('Evidence (working memory) databases have been (re-)initiatlised.')

    def evidence(self):
        e_preds = []
        e_confs = []

        for e in self.s1_db_s:
            e_preds.append(e[0])
            e_confs.append(e[1])

        return e_preds, e_confs

    def s1_reason(self):
        self.logger.log_mini_header('Evidence State (S1) (H)')
        self.s1_db_h.write()
        self.logger.log_mini_header('Evidence State (S1) (S)')
        self.s1_db_s.write()
        self.s1_reason_h()
        self.s1_reason_s()
        self.s1_predictions_in_segment = self.s1_predictions_in_segment + 1

    def s2_reason(self):
        self.logger.log_mini_header('Evidence State (S2) (H)')
        self.s2_db_h.write()
        self.logger.log_mini_header('Evidence State (S2) (S)')
        self.s2_db_s.write()
        self.s2_reason_h()
        self.s2_reason_s()
        self.s2_predictions_in_segment = self.s2_predictions_in_segment + 1

    def s1_reason_h(self):
        result = MLNQuery(mln=self.mln_h, db=self.s1_db_h, method='MC-SAT').run() # EnumerationAsk

        queries = []
        for cla in self.ahh.get_parents():
            predicate = 'class(S,' + cla + ')'
            queries.append(predicate)

        probs = []
        for q in queries:
            try:
                prob = result.results[q]
            except:
                prob = 0.0
            probs.append(prob)
        
        winner = np.argmax(probs)

        parents = self.ahh.get_parents()
        self.logger.log_mini_header('Query Results (H)')
        for i in range(0, len(probs)):
            msg = '(' + parents[i] + ',' + str(probs[i]) + ')'
            self.logger.log(msg)
        msg = 'Prediction (H): (' + parents[winner] + ',' + str(probs[winner]) + ')'
        self.logger.log_great(msg)

        self.s1_predictions_h.append((parents[winner], probs[winner], parents, probs))

    def s2_reason_h(self):
        result = MLNQuery(mln=self.mln_h, db=self.s2_db_h, method='MC-SAT').run() # EnumerationAsk

        queries = []
        for cla in self.ahh.get_parents():
            predicate = 'class(S,' + cla + ')'
            queries.append(predicate)

        probs = []
        for q in queries:
            try:
                prob = result.results[q]
            except:
                prob = 0.0
            probs.append(prob)
        
        winner = np.argmax(probs)

        parents = self.ahh.get_parents()
        self.logger.log_mini_header('Query Results (H)')
        for i in range(0, len(probs)):
            msg = '(' + parents[i] + ',' + str(probs[i]) + ')'
            self.logger.log(msg)
        msg = 'Prediction (H): (' + parents[winner] + ',' + str(probs[winner]) + ')'
        self.logger.log_great(msg)

        self.s2_predictions_h.append((parents[winner], probs[winner], parents, probs))
    
    def s1_reason_s(self):
        result = MLNQuery(mln=self.mln_s, db=self.s1_db_s, method='MC-SAT').run() # EnumerationAsk

        queries = []
        for cla in self.ahh.get_children(valid_room=self.room_e_history[-1]):
            predicate = 'class(S,' + cla + ')'
            queries.append(predicate)

        probs = []
        for q in queries:
            try:
                prob = result.results[q]
            except:
                prob = 0.0
            probs.append(prob)
        
        winner = np.argmax(probs)

        children = self.ahh.get_children(valid_room=self.room_e_history[-1])
        self.logger.log_mini_header('Query Results (S)')
        for i in range(0, len(probs)):
            msg = '(' + children[i] + ',' + str(probs[i]) + ')'
            self.logger.log(msg)
        msg = 'Prediction (S): (' + children[winner] + ',' + str(probs[winner]) + ')'
        self.logger.log_great(msg)

        self.s1_predictions_s.append((children[winner], probs[winner], children, probs))

    def s2_reason_s(self):
        result = MLNQuery(mln=self.mln_s, db=self.s2_db_s, method='MC-SAT').run() # EnumerationAsk

        queries = []
        for cla in self.ahh.get_children(valid_room=self.room_e_history[-1]):
            predicate = 'class(S,' + cla + ')'
            queries.append(predicate)

        probs = []
        for q in queries:
            try:
                prob = result.results[q]
            except:
                prob = 0.0
            probs.append(prob)
        
        winner = np.argmax(probs)

        children = self.ahh.get_children(valid_room=self.room_e_history[-1])
        self.logger.log_mini_header('Query Results (S)')
        for i in range(0, len(probs)):
            msg = '(' + children[i] + ',' + str(probs[i]) + ')'
            self.logger.log(msg)
        msg = 'Prediction (S): (' + children[winner] + ',' + str(probs[winner]) + ')'
        self.logger.log_great(msg)

        self.s2_predictions_s.append((children[winner], probs[winner], children, probs))

    def get_prevailing_predictions(self):
        # H
        if self.s2_active:
            s1_prediction = self.s1_predictions_h[-1]
            s2_prediction = self.s2_predictions_h[-1]

            prevailing_prediction_h = None
            if (s1_prediction[1] > MIN_SEGMENT_CONF) and (s2_prediction[1] > MIN_SEGMENT_CONF):
                if s2_prediction[1] > s1_prediction[1]:
                    prevailing_prediction_h = s2_prediction[0]
                else:
                    prevailing_prediction_h = s1_prediction[0]

            # S
            s1_prediction = self.s1_predictions_s[-1]
            s2_prediction = self.s2_predictions_s[-1]

            prevailing_prediction_s = None
            if (s1_prediction[1] > MIN_SEGMENT_CONF) and (s2_prediction[1] > MIN_SEGMENT_CONF):
                if s2_prediction[1] > s1_prediction[1]:
                    prevailing_prediction_s = s2_prediction[0]
                else:
                    prevailing_prediction_s = s1_prediction[0]
        else:
            prevailing_prediction_h = self.s1_predictions_h[-1][0]
            prevailing_prediction_s = self.s1_predictions_s[-1][0]

        return prevailing_prediction_h, prevailing_prediction_s

    def add(self, evidence, etype, room):
        resp = 'OK. Attempting to add: (' + etype + ',' + evidence + ')'

        if self.valid_pred(evidence, etype):
            self.add_pred(evidence, etype, room)
            if self.mode == 'predict':
                self.predict_next_cycle = True
            resp = 'OK. Added: (' + etype + ',' + evidence + ')'
            self.last_add = etype

            self.ros_move_to_room(room)
        else:
            resp = 'Invalid evidence type or invalid predicate. Valid types are: event.'

        self.logger.log(resp)
        return resp

    def add_pred(self, evidence, etype, room):
        pred = 'involves(S,' + evidence + ')'
        self.current_evidence = evidence

        if self.mode == 'train':
            self.asm.action('train', 'human', evidence, prediction=None)
            self.ps['events'].append(pred)
        elif self.mode == 'predict':
            self.s1_db_h[pred] = 1.0
            self.s1_db_s[pred] = 1.0
            if self.s2_active:
                self.s2_db_h[pred] = 1.0
                self.s2_db_s[pred] = 1.0
            self.room_e_history.append(room)
            self.pred_e_history.append(pred)
            canceller, persistent = self.is_canceller(pred)
            if canceller:
                self.s1_db_h[persistent] = 0.0
                self.s1_db_s[persistent] = 0.0
                self.s2_db_h[persistent] = 0.0
                self.s2_db_s[persistent] = 0.0

        if self.mode == 'predict':
            if len(self.pred_e_history) > 1:
                previous = self.pred_e_history[-2]
                previous = previous.split(',')
                previous = previous[1]
                previous = previous.rstrip(')')
                pred = 'after(' + previous+ ',' + evidence + ')'
                print(pred)
                self.s1_db_h[pred] = 1.0
                self.s1_db_s[pred] = 1.0
                if self.s2_active:
                    self.s2_db_h[pred] = 1.0
                    self.s2_db_s[pred] = 1.0

    def valid_pred(self, evidence, etype):
        if etype in etypes:
            if evidence in self.known_events:
                return True
            else:
                return False
        else:
            return False

    def delete(self, evidence, etype):
        resp = 'OK'

        if etype  == 'event':
            if evidence in self.events:
                predicate = 'involves(S,' + evidence + ')'
                self.s1_db_h[predicate] = 0.0
                self.s1_db_s[predicate] = 0.0
                if self.s2_active:
                    self.s2_db_h[predicate] = 0.0
                    self.s2_db_s[predicate] = 0.0
            else:
                resp = 'Evidence not modelled in MLN.'
        else:
            resp = 'Invalid evidence type. Valid types are: event' 

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
        config['grammar'] = 'PRACGrammar'
        config['logic'] = 'FirstOrderLogic'
        config['method'] = 'DBPLL_CG'
        config['epreds'] = ['involves', 'occurs', 'after']
        config['qpreds'] = 'class'
        config['optimizer'] = 'bfgs'
        config['multicore'] = False
        config['profile'] = 0
        config['shuffle'] = 0
        config['prior_mean'] = 0.5
        config['prior_stddev'] = 1
        config['save'] = True
        config['use_initial_weights'] = 0
        config['use_prior'] = 0

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
        config['grammar'] = 'PRACGrammar'
        config['logic'] = 'FirstOrderLogic'
        config['method'] = 'DBPLL_CG'
        config['epreds'] = ['involves', 'occurs', 'after']
        config['qpreds'] = 'class'
        config['optimizer'] = 'bfgs'
        config['multicore'] = False
        config['profile'] = 0
        config['shuffle'] = 0
        config['prior_mean'] = 0.5
        config['prior_stddev'] = 1
        config['save'] = True
        config['use_initial_weights'] = 0
        config['use_prior'] = 0

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
            if len(self.s1_predictions_s) < 1:
                self.logger.log('Cannot decay. No prediction has been made yet.')
            else:
                rooms = self.ahh.get_rooms(self.s1_predictions_s[-1][0])
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
            self.s2_clear_pred_history()
            self.s1_clear_pred_history()
            self.s2_active = False

        self.s1_reason()
        
        if not clear_e_pred_history and not self.s2_active and self.s1_predictions_in_segment > MIN_SEGMENT_DEPTH:
            s1_consistent_h, s1_consistent_s = self.tdch.is_consistent(self.s1_predictions_h[-2][0], self.s1_predictions_s[-2][0], self.pred_e_history[-1])
            self.s1_prediction_h_before_s2 = self.s1_predictions_h[-2][0]
            self.s1_prediction_s_before_s2 = self.s1_predictions_s[-2][0]

            if not s1_consistent_s:
                self.logger.log('Inconsistent event. Now considering that a new activity has occured...')
                self.s2_active = True
                self.asm.set_s2_active(True)

                self.s2_db_h[self.pred_e_history[-1]] = 1.0
                self.s2_db_s[self.pred_e_history[-1]] = 1.0

        s1_consistent_s = None
        s2_consistent_s = None
        agree = None
        if self.s2_active:
            self.s2_reason()

            if self.s2_predictions_in_segment > MIN_SEGMENT_DEPTH: # OR MIN_SEGMENT_DEPTH - 1
                s1_consistent_h, s1_consistent_s = self.tdch.is_consistent(self.s1_prediction_h_before_s2, self.s1_prediction_s_before_s2, self.pred_e_history[-1])
                s2_consistent_h, s2_consistent_s = self.tdch.is_consistent(self.s2_predictions_h[-1][0], self.s2_predictions_s[-1][0], self.pred_e_history[-1])

                if self.s1_predictions_s[-1][0] == self.s2_predictions_s[-1][0]:
                    agree = True
                else:
                    agree = False

                log = 'Consistency checking (s1_consistent_s, s2_consistent_s, agree): ' + str(s1_consistent_s) + ', ' + str(s2_consistent_s) + ', ' + str(agree)
                self.logger.log(log)

                # check whether we can finalise segment
                self.s1_consistent_agreement.append(s1_consistent_s)
                self.s2_consistent_agreement.append(s2_consistent_s)
                s2_conf = self.s2_predictions_s[-1][1]

                s1_count = 0
                for entry in self.s1_consistent_agreement:
                    if entry == False:
                        s1_count = s1_count + 1
                if s1_count == len(self.s1_consistent_agreement):
                    s1_all_false = True
                else:
                    s1_all_false = False

                s2_count = 0
                for entry in self.s2_consistent_agreement:
                    if entry == True:
                        s2_count = s2_count + 1
                if s2_count == len(self.s2_consistent_agreement):
                    s2_all_true = True
                else:
                    s2_all_true = False

                if s1_all_false and s2_all_true and s2_conf > MIN_SEGMENT_CONF:
                    self.swap_s2_to_s1()
                else:
                    if not self.query_triggered:
                        self.qs.query_select(self.s1_predictions_h[-1][0], self.s1_predictions_s[-1][0], self.s2_predictions_h[-1][0], self.s2_predictions_s[-1][0], s1_consistent_s, s2_consistent_s, agree)
                        self.segment_save_e = None
                        self.query_triggered = True

    def s1_clear_pred_history(self):
        self.segment_save_e = copy.deepcopy(self.pred_e_history)
        self.segment_save_e.pop()

        new_room = self.room_e_history[-1]
        new_pred = self.pred_e_history[-1]
        for i in range(0, len(self.pred_e_history) - 1):
            if not self.is_persistent(self.pred_e_history[i]):
                self.s1_db_h[self.pred_e_history[i]] = 0.0
                self.s1_db_s[self.pred_e_history[i]] = 0.0
        self.s1_db_h[new_pred] = 1.0
        self.s1_db_s[new_pred] = 1.0
        
        self.room_e_history = []
        self.room_e_history.append(new_room)

        self.pred_e_history = []
        self.pred_e_history.append(new_pred)

        self.s1_predictions_in_segment = 0

        self.asm.start_sequence('predict')

    def s2_clear_pred_history(self):
        for i in range(0, len(self.pred_e_history)):
            self.s2_db_h[self.pred_e_history[i]] = 0.0
            self.s2_db_s[self.pred_e_history[i]] = 0.0

        self.s2_predictions_in_segment = 0

        self.query_triggered = False

        self.asm.set_s2_active(False)

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

    def swap_s2_to_s1(self):
        if self.s2_active:
            print('Swapping S2 to S1...')
            self.s1_db_h = copy.deepcopy(self.s2_db_h)
            self.s1_db_s = copy.deepcopy(self.s2_db_s)

            self.s1_predictions_h = copy.deepcopy(self.s2_predictions_h)
            self.s1_predictions_s = copy.deepcopy(self.s2_predictions_s)

            self.pred_e_history = self.pred_e_history[-self.s2_predictions_in_segment:]

            self.s1_predictions_in_segment = copy.deepcopy(self.s2_predictions_in_segment)

            self.s2_clear_pred_history()

            self.s1_consistent_agreement = []
            self.s2_consistent_agreement = []
            
            self.s2_active = False

            self.asm.swap_s2_to_s1()

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

    @app.route('/train_start', methods = ['POST'])
    def train_start_handler():
        m.ros_add_rule_start_callback(None)

        return 'OK'

    @app.route('/train_stop', methods = ['POST'])
    def train_stop_handler():
        m.ros_add_rule_stop_callback(None)

        return 'OK'

    @app.route('/train_label', methods = ['POST'])
    def train_label_handler():
        data = request.get_json()
        label = data['label']

        m.ros_add_rule_label_callback_internal(label)

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
        pred, prob = m.s1_reason()

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
