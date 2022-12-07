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

from adl_helper import ADLHelper
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
        self.tdch = TrainDBConsistencyHelper(self.rel_path)
        self.kdh = KnownDomainsHelper(self.rel_path)
        if RESET:
            self.ah = ADLHelper(self.rel_path, reset=True)
            self.asm = ADLSequenceModeller(self.rel_path, reset=True)
        else:
            self.ah = ADLHelper(self.rel_path, reset=False)
            self.asm = ADLSequenceModeller(self.rel_path, reset=False)
        self.asm.start_sequence('predict')

        # Reset Global DB
        if RESET:
            path = self.rel_path + '/src/DBs/global_DB_s.txt'
            try:
                os.remove(path)
            except FileNotFoundError:
                pass
            open(path, 'a').close()

        # MLN Name
        self.mln_prefix = 'default'

        # Predictions
        self.s1_predictions_s = []
        self.s2_predictions_s = []

        # Segmentation
        self.current_room = None
        self.current_evidence = None # used by ASM
        self.room_e_history = []
        self.pred_e_history = []
        self.pred_a_history = []
        self.last_add = None
        self.s2_active = False
        self.s1_predictions_in_segment = 0
        self.s2_predictions_in_segment = 0
        self.query_triggered = False
        self.label_save = None
        self.segment_save_e = None
        self.s1_prediction_s_before_s2 = None
        self.s1_consistent_agreement = []
        self.s2_consistent_agreement = []

        # Loads
        self.known_events, self.known_times, self.known_adls = self.kdh.count()
        self.create_pred_store()
        self.load_knowledge_bases()
        self.create_default_mlns()
        self.init_groundings()
        self.skip_train = False
        self.load_global_train_dbs()
        if RESET:
            self.logger.log_warn('WARNING: The MLNs have not been trained. Predictions will be non-sensical until training data has been provided.')
        else:
            if not self.skip_train:
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

        # ROS Publishers
        self.pub_ros_predictions = rospy.Publisher('/robot_har_mln/predictions', har_predictions, queue_size=10)
        self.pub_ros_evidence = rospy.Publisher('/robot_har_mln/db/evidence', har_evidence_list, queue_size=10)
        self.pub_ros_move_to_room = rospy.Publisher('/robot_har_help_service/move_to_room/request', String, queue_size=10)
        
        # ROS Action Servers
        self.action_name = 'robot_har_mln/har_reason'
        self._as = actionlib.SimpleActionServer(self.action_name, ronsm_messages.msg.har_reasonAction, execute_cb=self.ros_reason_callback)
        self._as.start()

        self.logger.log_great('Ready.')

        if RESET:
            self.logger.log_warn('Training must be performed while started in normal mode (RESET = False). Exiting...')
            exit()

    # Init. Methods

    def create_default_mlns(self):
        self.mln_s = MLN(grammar='PRACGrammar', logic='FirstOrderLogic')

        self.init_mlns(default_rules=True)

        self.save_mlns()

        self.reset_working_memory()

    def load_global_train_dbs(self):
        path = self.rel_path + '/src/DBs/' + 'global' + '_DB_s.txt'
        if os.stat(path).st_size == 0:
            self.logger.log_warn('Training database is empty, system will not train until samples added.')
            self.skip_train = True
        self.global_train_db_s = Database.load(self.mln_s, dbfiles=path)

    def load_knowledge_bases(self):
        base_path = self.rel_path + '/src/MLNs/common/'

        event_path = base_path + 'event.txt'
        time_path = base_path + 'time.txt'
        predicate_path = base_path + 'predicate.txt'
        rules_s_path = base_path + 'rules.txt'

        event_file = open(event_path, 'r')
        time_file = open(time_path, 'r')
        predicate_file = open(predicate_path, 'r')
        rules_s_file = open(rules_s_path, 'r')

        self.events = []
        self.times = []
        self.events_persist = []
        self.predicates = []
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

        for line in rules_s_file:
            self.rules_s.append(line.rstrip('\n'))

    def init_mlns(self, default_rules=False):
        is_fuzzy = False
        for predicate in self.predicates:
            if is_fuzzy:
                self.mln_s << '#fuzzy \n' + predicate
                is_fuzzy = False
            else:
                self.mln_s << predicate
            if predicate == '#fuzzy':
                is_fuzzy = True

        if default_rules:
            for rule in self.rules_s:
                self.mln_s << rule

        self.print_mlns()

    def init_groundings(self):
        self.s1_db_s = Database(self.mln_s)
        
        for event in self.known_events:
            predicate = 'involves(S,' + event + ')'
            self.s1_db_s << predicate
            self.s1_db_s[predicate] = 0.0
            self.s2_db_s << predicate
            self.s2_db_s[predicate] = 0.0

        for time in self.known_times:
            predicate = 'occurs(S, ' + time + ')'
            self.s1_db_s << predicate
            self.s1_db_s[predicate] = 0.0
            self.s2_db_s << predicate
            self.s2_db_s[predicate] = 0.0

        for event_o in self.known_events:
            for event_i in self.known_events:
                predicate = 'after(' + event_o + ',' + event_i + ')'
                self.s1_db_s << predicate
                self.s1_db_s[predicate] = 0.0
                self.s2_db_s << predicate
                self.s2_db_s[predicate] = 0.0

    def create_pred_store(self):
        self.ps = {}
        self.ps['events'] = []
        self.ps['rooms'] = []
        self.ps['after'] = []
        
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
                    prediction = self.get_prevailing_prediction()
                    self.asm.action('predict', 'human', self.current_evidence, prediction=prediction)
                    self.ros_pub_predictions(prediction)
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
        if ((room != self.current_room) or (self.current_room == None)) and not self.asm.locked:
            log = 'Sending robot to room: ' + room
            self.logger.log(log)
            msg = String()
            msg.data = room
            self.pub_ros_move_to_room.publish(msg)
            self.current_room = room

    # ROS Publishers

    def ros_pub_predictions(self, prediction):
        msg = har_predictions()
        msg.prediction = prediction
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
        self.logger.log_mini_header('MLN_S')
        self.mln_s.write()

    def save_mlns(self):
        path = self.rel_path + '/src/MLNs/' + self.mln_prefix + '_MLN_s.mln'
        self.mln_s.tofile(path)

    def save_rule(self, label):
        self.asm.stop_sequence('train')
        self.asm.label_sequence('train', label)

        self.ah.update_rooms(label, self.ps['rooms'])

        self.save_evidence(label)

        del self.kdh
        self.kdh = KnownDomainsHelper(self.rel_path)
        self.known_events, self.known_times, self.known_adls = self.kdh.count()

        self.create_pred_store()

        self.train_mlns()

        self.save_mlns()

    def save_evidence(self, label):
        path_s = self.rel_path + '/src/DBs/' + 'global' + '_DB_s.txt'
        file = open(path_s, 'a')

        for event in self.ps['events']:
            file.write(event + '\n')
        for after in self.ps['after']:
            file.write(after + '\n')
        sample_label = 'class(S,' + label + ')'
        file.write(sample_label + '\n')
        file.write('---\n')

        file.close()

        self.load_global_train_dbs()

    def save_segment(self):
        label = self.label_save

        self.asm.stop_sequence('train')
        self.asm.label_sequence('train', label)

        segment_e = self.segment_save_e
        segment_e = set(segment_e)
        segment_e = list(segment_e)

        segment_a = self.segment_save_a
        segment_a = set(segment_a)
        segment_a = list(segment_a)

        self.save_evidence_query(segment_e, segment_a, label)

        self.label_save = None
        self.segment_save_e = None
        self.segment_save_a = None

        self.train_mlns()

        self.save_mlns()

    def save_evidence_query(self, segment_e, segment_a, label):
        path_s = self.rel_path + '/src/DBs/' + 'global' + '_DB_s.txt'
        file = open(path_s, 'a')

        for event in segment_e:
            file.write(event + '\n')
        for after in segment_a:
            file.write(after + '\n')
        sample_label = 'class(S,' + label + ')'
        file.write(sample_label + '\n')
        file.write('---\n')

        file.close()

        self.load_global_train_dbs()
    
    def reset_working_memory(self):
        # DBs
        try:
            del self.s1_db_s
            del self.ps
        except:
            self.logger.log_warn('Evidence (working memory) databases have not yet be created, they will now be created...')

        self.s1_db_s = Database(self.mln_s)
        self.s2_db_s = Database(self.mln_s)

        # Predictions
        self.s1_predictions_s = []
        self.s2_predictions_s = []

        # Segmentation
        self.room_e_history = []
        self.pred_e_history = []
        self.pred_a_history = []
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
        self.logger.log_mini_header('Evidence State (S1) (S)')
        self.s1_db_s.write()
        self.s1_reason_s()
        self.s1_predictions_in_segment = self.s1_predictions_in_segment + 1

    def s2_reason(self):
        self.logger.log_mini_header('Evidence State (S2) (S)')
        self.s2_db_s.write()
        self.s2_reason_s()
        self.s2_predictions_in_segment = self.s2_predictions_in_segment + 1
    
    def s1_reason_s(self):
        result = MLNQuery(mln=self.mln_s, db=self.s1_db_s, method='MC-SAT').run() # EnumerationAsk

        queries = []

        adls = self.ah.get_adls()

        for adl in adls:
            predicate = 'class(S,' + adl + ')'
            queries.append(predicate)

        probs = []
        for q in queries:
            try:
                prob = result.results[q]
            except:
                prob = 0.0
            probs.append(prob)
        
        winner = np.argmax(probs)

        self.logger.log_mini_header('Query Results (S)')
        for i in range(0, len(probs)):
            msg = '(' + adls[i] + ',' + str(probs[i]) + ')'
            self.logger.log(msg)
        msg = 'Prediction (S): (' + adls[winner] + ',' + str(adls[winner]) + ')'
        self.logger.log_great(msg)

        self.s1_predictions_s.append((adls[winner], probs[winner], adls, probs))

    def s2_reason_s(self):
        result = MLNQuery(mln=self.mln_s, db=self.s2_db_s, method='MC-SAT').run() # EnumerationAsk

        queries = []

        adls = self.ah.get_adls()

        for adl in adls:
            predicate = 'class(S,' + adl + ')'
            queries.append(predicate)

        probs = []
        for q in queries:
            try:
                prob = result.results[q]
            except:
                prob = 0.0
            probs.append(prob)
        
        print(probs)
        winner = np.argmax(probs)

        self.logger.log_mini_header('Query Results (S)')
        for i in range(0, len(probs)):
            msg = '(' + adls[i] + ',' + str(probs[i]) + ')'
            self.logger.log(msg)
        msg = 'Prediction (S): (' + adls[winner] + ',' + str(adls[winner]) + ')'
        self.logger.log_great(msg)

        self.s2_predictions_s.append((adls[winner], probs[winner], adls, probs))

    def get_prevailing_prediction(self):
        # H
        if self.s2_active:
            s1_prediction = self.s1_predictions_s[-1]
            s2_prediction = self.s2_predictions_s[-1]

            prevailing_prediction = None
            if (s1_prediction[1] > MIN_SEGMENT_CONF) and (s2_prediction[1] > MIN_SEGMENT_CONF):
                if s2_prediction[1] > s1_prediction[1]:
                    prevailing_prediction = s2_prediction[0]
                else:
                    prevailing_prediction = s1_prediction[0]
        else:
            prevailing_prediction = self.s1_predictions_s[-1][0]

        return prevailing_prediction

    def add(self, evidence, etype, room):
        resp = 'OK. Attempting to add: (' + etype + ',' + evidence + ')'

        if evidence == 'Presence':
            log = 'Not modelling using presence events!'
            self.logger.log_warn(log)
            return

        if self.mode == 'train':
            self.add_pred(evidence, etype, room)
            resp = 'OK. Added: (' + etype + ',' + evidence + ')'
            self.last_add = etype

            # self.ros_move_to_room(room)
        elif self.mode == 'predict':
            if self.valid_pred(evidence, etype):
                self.add_pred(evidence, etype, room)
                self.predict_next_cycle = True
                resp = 'OK. Added: (' + etype + ',' + evidence + ')'
                self.last_add = etype

                # self.ros_move_to_room(room)
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
            self.ps['rooms'].append(room)
        elif self.mode == 'predict':
            self.s1_db_s[pred] = 1.0
            if self.s2_active:
                self.s2_db_s[pred] = 1.0
            self.room_e_history.append(room)
            self.pred_e_history.append(pred)
            canceller, persistent = self.is_canceller(pred)
            if canceller:
                self.s1_db_s[persistent] = 0.0
                self.s2_db_s[persistent] = 0.0

        if self.mode == 'train':
            if len(self.ps['events']) > 1:
                previous = self.ps['events'][-2]
                previous = previous.split(',')
                previous = previous[1]
                previous = previous.rstrip(')')
                pred = 'after(' + previous + ',' + evidence + ')'
                self.ps['after'].append(pred)
        elif self.mode == 'predict':
            if len(self.pred_e_history) > 1:
                previous = self.pred_e_history[-2]
                previous = previous.split(',')
                previous = previous[1]
                previous = previous.rstrip(')')
                pred = 'after(' + previous+ ',' + evidence + ')'
                self.s1_db_s[pred] = 1.0
                if self.s2_active:
                    self.s2_db_s[pred] = 1.0
                self.pred_a_history.append(pred)

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
                self.s1_db_s[predicate] = 0.0
                if self.s2_active:
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
                rooms = self.ah.get_rooms(self.s1_predictions_s[-1][0])
                if len(rooms) == 0:
                    self.logger.log('Not decaying any predicates, location is consistent.')
                else:
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
            s1_consistent_s = self.tdch.is_consistent(self.s1_predictions_s[-2][0], self.pred_e_history[-1])
            self.s1_prediction_s_before_s2 = self.s1_predictions_s[-2][0]

            if not s1_consistent_s:
                self.logger.log('Inconsistent event. Now considering that a new activity has occured...')
                self.s2_active = True
                self.asm.set_s2_active(True)

                self.s2_db_s[self.pred_e_history[-1]] = 1.0

        s1_consistent_s = None
        s2_consistent_s = None
        agree = None
        if self.s2_active:
            self.s2_reason()

            if self.s2_predictions_in_segment > MIN_SEGMENT_DEPTH: # OR MIN_SEGMENT_DEPTH - 1
                s1_consistent_s = self.tdch.is_consistent(self.s1_prediction_s_before_s2, self.pred_e_history[-1])
                s2_consistent_s = self.tdch.is_consistent(self.s2_predictions_s[-2][0], self.pred_e_history[-1])

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
                        self.qs.query_select(self.s1_predictions_s[-1][0], self.s2_predictions_s[-1][0], s1_consistent_s, s2_consistent_s, agree)
                        self.segment_save_e = None
                        self.query_triggered = True

    def s1_clear_pred_history(self):
        self.segment_save_e = copy.deepcopy(self.pred_e_history)
        self.segment_save_a = copy.deepcopy(self.pred_a_history)
        self.segment_save_e.pop()

        new_room = self.room_e_history[-1]
        new_pred = self.pred_e_history[-1]
        if len(self.pred_a_history) > 0:
            new_after = self.pred_a_history[-1]

        for i in range(0, len(self.pred_e_history) - 1):
            if not self.is_persistent(self.pred_e_history[i]):
                self.s1_db_s[self.pred_e_history[i]] = 0.0
        self.s1_db_s[new_pred] = 1.0

        if len(self.pred_a_history) > 0:
            for i in range(0, len(self.pred_a_history) - 1):
                self.s1_db_s[self.pred_a_history[i]] = 0.0
            self.s1_db_s[new_after]
        
        self.room_e_history = []
        self.room_e_history.append(new_room)

        self.pred_e_history = []
        self.pred_e_history.append(new_pred)

        self.pred_a_history = []
        if len(self.pred_a_history) > 0:
            self.pred_a_history.append(new_after)    

        self.s1_predictions_in_segment = 0

        self.asm.start_sequence('predict')

    def s2_clear_pred_history(self):
        for i in range(0, len(self.pred_e_history)):
            self.s2_db_s[self.pred_e_history[i]] = 0.0

        for i in range(0, len(self.pred_a_history)):
            self.s2_db_s[self.pred_a_history[i]] = 0.0

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
            self.s1_db_s = copy.deepcopy(self.s2_db_s)

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
