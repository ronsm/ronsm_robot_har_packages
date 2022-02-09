#! /usr/bin/env python3

import os
import threading

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

from std_msgs import msg
from std_msgs.msg import String

from ronsm_messages.msg import har_simple_evidence
from ronsm_messages.msg import har_reset
from ronsm_messages.msg import har_evidence_list
import ronsm_messages.msg

from adl_hierarchy_helper import ADLHierarchyHelper

etypes = ['event', 'percept']

class Main():
    _ros_reason_feedback = ronsm_messages.msg.har_reasonFeedback()
    _ros_reason_result = ronsm_messages.msg.har_reasonResult()
    
    def __init__(self):
        rospy.init_node('robot_har_mln', disable_signals=True)

        # Modes
        self.mode = 'predict' # default is predict
        self.train_mode = 'global'

        # ROSPack Path
        rospack = rospkg.RosPack()
        self.rel_path = rospack.get_path('robot_har_mln')

        # ADL Hierarchy Helper
        self.adlhh = ADLHierarchyHelper(self.rel_path)

        # MLN Name
        self.mln_prefix = 'default'

        # ROS Subscribers
        self.sub_sel_evidence = rospy.Subscriber('/ralt_semantic_event_publisher/simple', har_simple_evidence, callback=self.ros_evidence_callback)
        self.sub_ros_evidence = rospy.Subscriber('/robot_har_mln/db/add_delete', har_simple_evidence, callback=self.ros_evidence_callback)
        self.sub_ros_reset = rospy.Subscriber('/robot_har_mln/db/reset', har_reset, callback=self.ros_reset_callback)
        self.sub_ros_new_mln = rospy.Subscriber('/robot_har_mln/mln/new', String, callback=self.ros_new_mln_callback)
        self.sub_ros_load_mln = rospy.Subscriber('/robot_har_mln/mln/load', String, callback=self.ros_load_mln_callback)
        self.sub_ros_add_rule_start = rospy.Subscriber('/robot_har_mln/mln/new_rule_start', String, callback=self.ros_add_rule_start_callback)
        self.sub_ros_add_rule_stop = rospy.Subscriber('/robot_har_mln/mln/new_rule_stop', String, callback=self.ros_add_rule_stop_callback)
        self.sub_ros_add_rule_label = rospy.Subscriber('/robot_har_mln/mln/label', String, callback=self.ros_add_rule_label_callback)
        self.pub_ros_mln_train = rospy.Subscriber('/robot_har_mln/mln/train', String, callback=self.ros_mln_train_callback)

        # ROS Publishers
        self.pub_ros_evidence = rospy.Publisher('/robot_har_mln/db/evidence', har_evidence_list, queue_size=10)
        
        # ROS Action Servers
        self.action_name = 'robot_har_mln/har_reason'
        self._as = actionlib.SimpleActionServer(self.action_name, ronsm_messages.msg.har_reasonAction, execute_cb=self.ros_reason_callback)
        self._as.start()

        # Loads
        self.create_pred_store()
        self.load_constants_and_percepts()
        self.create_default_mlns()
        self.init_percepts()
        self.load_global_train_dbs()
        self.train_mlns()

    # Init. Methods

    def create_default_mlns(self):
        self.mln_h = MLN(grammar='StandardGrammar', logic='FirstOrderLogic')
        self.mln_s = MLN(grammar='StandardGrammar', logic='FirstOrderLogic')

        self.init_mlns(default_rules=True)

        self.save_mlns()

        self.reset_current_evidence()

    def create_mlns(self, name):
        self.mln_prefix = name

        self.mln_h = MLN(grammar='StandardGrammar', logic='FirstOrderLogic')
        self.mln_s = MLN(grammar='StandardGrammar', logic='FirstOrderLogic')

        self.init_mlns()

        self.save_mlns()

        self.create_restricted_train_dbs()

        self.load_restricted_train_dbs()

        self.reset_current_evidence()

    def create_restricted_train_dbs(self):
        restricted_train_db_h = []
        path = self.rel_path + '/src/DBs/' + self.mln_prefix + '_DB_h.p'
        pickle.dump(restricted_train_db_h, open(path, 'wb'))

        restricted_train_db_s = []
        path = self.rel_path + '/src/DBs/' + self.mln_prefix + '_DB_s.p'
        pickle.dump(restricted_train_db_s, open(path, 'wb'))

    def load_global_train_dbs(self):
        path = self.rel_path + '/src/DBs/' + 'global' + '_DB_h.p'
        self.global_train_db_h = pickle.load(open(path, 'rb'))

        path = self.rel_path + '/src/DBs/' + 'global' + '_DB_s.p'
        self.global_train_db_s = pickle.load(open(path, 'rb'))

    def load_restricted_train_dbs(self):
        path = self.rel_path + '/src/DBs/' + self.mln_prefix + '_DB_h.p'
        self.restricted_train_db_h = pickle.load(open(path, 'rb'))

        path = self.rel_path + '/src/DBs/' + self.mln_prefix + '_DB_s.p'
        self.restricted_train_db_s = pickle.load(open(path, 'rb'))

    def load_constants_and_percepts(self):
        base_path = self.rel_path + '/src/MLNs/common/'

        self.action_name

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
        self.percepts = []
        self.predicates = []
        self.rules_h = []
        self.rules_s = []

        for line in event_file:
            self.events.append(line.rstrip('\n'))

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
        for activity in self.adlhh.get_parents():
            activity_str = activity_str + activity + ','
        activity_str = activity_str.rstrip(',') + '}'
        self.mln_h << activity_str
        
        # S
        activity_str = 'activity = {'
        for activity in self.adlhh.get_children():
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

    def load_mlns(self):
        path = self.rel_path + '/src/MLNs/' + self.mln_prefix + '_MLN_h.mln'
        self.mln_h = MLN.load(files=path, grammar='StandardGrammar', logic='FirstOrderLogic')

        path = self.rel_path + '/src/MLNs/' + self.mln_prefix + '_MLN_s.mln'
        self.mln_s = MLN.load(files=path, grammar='StandardGrammar', logic='FirstOrderLogic')
        
        self.load_restricted_train_dbs()
        self.reset_current_evidence()

    def init_percepts(self):
        self.db_h = Database(self.mln_h)
        self.db_s = Database(self.mln_s)

        for per in self.percepts:
            predicate = 'involves_percept(A,' + per + ')'
            self.db_h << predicate
            self.db_h[predicate] = 0.0
            self.db_s << predicate
            self.db_s[predicate] = 0.0

    def create_pred_store(self):
        self.ps = {}
        self.ps['events'] = []
        self.ps['percepts'] = []
        
    # ROS Loop 

    def loop(self):
        while(True):
            while not rospy.core.is_shutdown():
                self.decay()
                self.ros_publish()
                rospy.rostime.wallsleep(0.5)
            
    # ROS Callbacks

    def ros_evidence_callback(self, msg):
        print('[ROS] Received evidence:', msg.evidence, msg.etype)
        if msg.cmd == 'add':
            self.add(msg.evidence, msg.etype)
        elif msg.cmd == 'delete':
            self.delete(msg.evidence, msg.etype)
        else:
            print('[ROS] Invalid command in ROS evidence topic.')
            
    def ros_reason_callback(self, goal):
        print('[ROS] Received requested to reason.')
        pred, conf = self.reason()
        
        self._result.pred = pred
        self._result.conf = conf
        
        self._as.set_succeeded(self._result)
        
    def ros_reset_callback(self, msg):
        print('[ROS] Received reset command.')
        self.reset_current_evidence()

    def ros_new_mln_callback(self, msg):
        print('[ROS] Received request to create new MLNs:', msg.data)
        name = msg.data

        self.create_mlns(name)

    def ros_load_mln_callback(self, msg):
        print('[ROS] Received request to load MLNs:', msg.data)
        name = msg.data

        self.mln_prefix = name

        self.load_mlns()

    def ros_add_rule_start_callback(self, msg):
        self.mode = 'train'
        print('[ROS] Entered training mode.')

    def ros_add_rule_stop_callback(self, msg):
        self.mode = 'predict'
        print('[ROS] Entered predict mode.')

    def ros_add_rule_label_callback(self, msg):
        label = msg.data
        print('[ROS] Received label for new rule:', label)
        self.save_rule(label)

    def ros_mln_train_callback(self, msg):
        mode = msg.data
        print('[ROS] Changing MLN training mode to:', mode)
        if mode == 'global':
            self.train_mode = 'global'
        elif mode == 'restricted':
            self.train_mode = 'restricted'
        else:
            print('[ROS] Invalid MLN train mode specified.')
            
        print('[ROS] Received request to train MLNs.')
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
        print('[INFO] * * * mln_h * * *')
        self.mln_h.write()
        print('[INFO] * * * mln_s * * *')
        self.mln_s.write()

    def save_mlns(self):
        path = self.rel_path + '/src/MLNs/' + self.mln_prefix + '_MLN_h.mln'
        self.mln_h.tofile(path)

        path = self.rel_path + '/src/MLNs/' + self.mln_prefix + '_MLN_s.mln'
        self.mln_s.tofile(path)

    def save_train_dbs(self):
        path = self.rel_path + '/src/DBs/' + 'global' + '_DB_h.p'
        pickle.dump(self.global_train_db_h, open(path, 'wb'))

        path = self.rel_path + '/src/DBs/' + 'global' + '_DB_s.p'
        pickle.dump(self.global_train_db_s, open(path, 'wb'))

        path = self.rel_path + '/src/DBs/' + self.mln_prefix + '_DB_h.p'
        pickle.dump(self.restricted_train_db_h, open(path, 'wb'))

        path = self.rel_path + '/src/DBs/' + self.mln_prefix + '_DB_s.p'
        pickle.dump(self.restricted_train_db_s, open(path, 'wb'))

    def print_train_dbs(self):
        print('[INFO] * * * global_train_db_h * * *')
        print(self.global_train_db_h)
        print('[INFO] * * * global_train_db_s * * *')
        print(self.global_train_db_s)

    def save_rule(self, label):
        label_s = label
        label_h = self.adlhh.get_parent(label_s)

        rule_str = '0.0 '
        for event in self.ps['events']:
            rule_str = rule_str + event[0] + '(a,' + event[1] + ')'
            rule_str = rule_str + ' ^ '
        rule_str = rule_str.rstrip(' ^ ')
        
        rule_str_h = rule_str + ' => ' + 'class(a,' + label_h + ')'
        rule_str_s = rule_str + ' => ' + 'class(a,' + label_s + ')'

        self.mln_h << rule_str_h
        self.mln_s << rule_str_s

        if len(self.ps['percepts']) > 0:
            rule_str = '0.0 '
            for percept in self.ps['percepts']:
                rule_str = rule_str + percept[0] + '(a,' + percept[1] + ')'
                rule_str = rule_str + ' ^ '
            rule_str = rule_str.rstrip(' ^ ')

            rule_str_s = rule_str + ' => ' + 'class(a,' + label_s + ')'

            self.mln_s << rule_str_s

        self.print_mlns()

        self.save_mlns()

        # Update training evidence DB
        evidence_and_label = [self.ps['events'], label_h]
        self.global_train_db_h.append(evidence_and_label)
        self.restricted_train_db_h.append(evidence_and_label)

        events_and_percepts = self.ps['events'] + self.ps['percepts']
        evidence_and_label = [events_and_percepts, label_s]
        self.global_train_db_s.append(evidence_and_label)
        self.restricted_train_db_s.append(evidence_and_label)

        self.print_train_dbs()

        self.save_train_dbs()

        self.create_pred_store()
    
    def reset_current_evidence(self):
        try:
            del self.db_h
            del self.db_s
            del self.ps
        except:
            print('[INFO] Current evidence DBs have not yet be created, they will now be created.')

        self.db_h = Database(self.mln_h)
        self.db_s = Database(self.mln_s)

        self.load_constants_and_percepts()
        self.init_percepts()
        self.create_pred_store()
        print('[MLN]', 'Reset.')

    def evidence(self):
        e_preds = []
        e_confs = []

        for e in self.db_s:
            e_preds.append(e[0])
            e_confs.append(e[1])

        return e_preds, e_confs

    def reason(self):
        result = MLNQuery(mln=self.mln_s, db=self.db_s, method='EnumerationAsk').run()

        queries = []
        for cla in self.activities:
            predicate = 'class(a,' + cla + ')'
            queries.append(predicate)

        probs = []
        for q in queries:
            prob = result.results[q]
            probs.append(prob)
        
        winner = np.argmax(probs)
        
        print('[MLN]', self.activities[winner], probs[winner])
        return self.activities[winner], probs[winner]

    def add(self, evidence, etype):
        resp = 'OK'

        if self.valid_pred(evidence, etype):
            self.add_pred(evidence, etype)
        else:
            print('[MLN] Invalid evidence type or invalid predicate. Valid types are: event or percept')

        print('[MLN]', resp)
        return resp

    def add_pred(self, evidence, etype):
        # if etype == 'event':
        #     pred = 'involves_event(a,' + evidence + ')'
        # elif etype == 'percept':
        #     pred = 'involves_percept(a,' + evidence + ')'
        if etype == 'event':
            pred = ("involves_event", evidence)
        elif etype == 'percept':
            pred = ("involves_percept", evidence)

        if self.mode == 'train':
            if etype == 'event':
                self.ps['events'].append(pred)
            elif etype == 'percept':
                self.ps['percepts'].append(pred)
        elif self.mode == 'predict':
            self.db_h << pred
            self.db_s << pred
    
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

        # simple model of deletion, instantly removes, need to implement
        # (i) instant deletion for events
        # (ii) time-based deletion for percepts (e.g. in 60 seconds rectractall will be called for predicate, use queue)
        if etype  == 'event':
            if evidence in self.events:
                predicate = 'involves_event(a,' + evidence + ')'
                self.db_h.retract(predicate)
                self.db_s.retract(predicate)
            else:
                resp = 'Evidence not modelled in MLN.'
        elif etype == 'percept':
            if evidence in self.percepts:
                predicate = 'involves_percept(a,' + evidence + ')'
                self.db_h.retract(predicate)
                self.db_s.retract(predicate)
            else:
                resp = 'Evidence not modelled in MLN.'
        else:
            resp = 'Invalid evidence type. Valid types are: event or percept' 

        print('[MLN]', resp)
        return resp

    def train_mlns(self):
        # Train H
        path_h = self.rel_path + '/src/temp/train_mln_h.txt'
        file = open(path_h, 'w')

        for i in range(0, len(self.global_train_db_h)):
            for event in self.global_train_db_h[i][0]:
                evidence = event[0] + '(S,' + event[1] + ')'
                file.write(evidence + '\n')
            evidence_label = 'class(S,' + self.global_train_db_h[i][1] + ')'
            file.write(evidence_label + '\n')
            file.write('---' + '\n')

        file.close()

        train_db_h = Database.load(self.mln_h, path_h)
        # train_db_h.write()

        # TODO: Deal with self.global_train_db_s

        DEFAULT_CONFIG = os.path.join(locs.user_data, global_config_filename)
        conf = PRACMLNConfig(DEFAULT_CONFIG)

        config = {}
        config['verbose'] = True
        config['discr_preds'] = 0
        config['db'] = train_db_h
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
        config['use_prior'] = 0

        config['infoInterval'] = 500
        config['resultsInterval'] = 1000
        conf.update(config)

        print('[MLN] Training...')
        learn = MLNLearn(conf, mln=self.mln_h, db=train_db_h)

        self.mln_h = learn.run()

        # Train S

        path_s = self.rel_path + '/src/temp/train_mln_s.txt'
        file = open(path_s, 'w')

        for i in range(0, len(self.global_train_db_s)):
            for event_percept in self.global_train_db_s[i][0]:
                evidence = event_percept[0] + '(S,' + event_percept[1] + ')'
                file.write(evidence + '\n')
            evidence_label = 'class(S,' + self.global_train_db_s[i][1] + ')'
            file.write(evidence_label + '\n')
            file.write('---' + '\n')

        file.close()

        train_db_s = Database.load(self.mln_s, path_s)

        for per in self.percepts:
            predicate = 'involves_percept(S,' + per + ')'
            self.db_h << predicate
            self.db_h[predicate] = 0.0
            self.db_s << predicate
            self.db_s[predicate] = 0.0

        DEFAULT_CONFIG = os.path.join(locs.user_data, global_config_filename)
        conf = PRACMLNConfig(DEFAULT_CONFIG)

        config = {}
        config['verbose'] = True
        config['discr_preds'] = 0
        config['db'] = train_db_s
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
        config['use_prior'] = 0

        config['infoInterval'] = 500
        config['resultsInterval'] = 1000
        conf.update(config)

        print('[MLN] Training...')
        learn = MLNLearn(conf, mln=self.mln_s, db=train_db_s)

        self.mln_s = learn.run()

        self.print_mlns()
    
        print('[MLN] Finished training.')
        

    # Additional Logic

    def decay(self):
        # do something here to decay events, e.g.
        # (i) events when added are put into a dictionary with a timestamp
        # (ii) every tick, we compare the timestamp of events with current timestamp
        # (iii) if timestamp is older than x (used a tiered system), then call delete on that event
        return

if __name__ == '__main__':
    m = Main()

    app = Flask(__name__)
    CORS(app)

    threading.Thread(target=lambda: m.loop()).start()

    @app.route('/reset', methods = ['POST'])
    def reset_handler():

        m.reset_current_evidence()

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

        resp = m.add(evidence, etype)

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

    app.run(host='0.0.0.0', port = 5010)
