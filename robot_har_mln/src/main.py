#! /usr/bin/env python3

import os
import threading

from pracmln import MLN, Database, MLNQuery
import numpy as np
from flask import Flask, request, jsonify
from flask_cors import CORS

import rospy
import rospkg
import actionlib

from std_msgs import msg
from std_msgs.msg import String

from ronsm_messages.msg import har_simple_evidence
from ronsm_messages.msg import har_reset
from ronsm_messages.msg import har_evidence_list
import ronsm_messages.msg

class Main():
    _ros_reason_feedback = ralt_signalman_messages.msg.har_reasonFeedback()
    _ros_reason_result = ralt_signalman_messages.msg.har_reasonResult()
    
    def __init__(self):
        rospy.init_node('robot_har_mln', disable_signals=True)

        self.sub_sel_evidence = rospy.Subscriber('/ralt_semantic_event_publisher/simple', har_simple_evidence, callback=self.ros_evidence_callback)
        self.sub_ros_evidence = rospy.Subscriber('/robot_har_mln/add_delete', har_simple_evidence, callback=self.ros_evidence_callback)
        self.sub_ros_reset = rospy.Subscriber('/robot_har_mln/reset', har_reset, callback=self.ros_reset_callback)

        self.pub_ros_evidence = rospy.Publisher('/robot_har_mln/evidence', har_evidence_list, queue_size=10)
        
        self.action_name = 'robot_har_mln/har_reason'
        self._as = actionlib.SimpleActionServer(self.action_name, ralt_signalman_messages.msg.har_reasonAction, execute_cb=self.ros_reason_callback)
        self._as.start()

        rospack = rospkg.RosPack()
        rel_path = rospack.get_path('robot_har_mln')
        path = rel_path + '/src/MLNs/kitchen_v1.mln'
        self.mln = MLN.load(files=path, grammar='StandardGrammar', logic='FuzzyLogic')
        self.db = Database(self.mln)

        self.load_events_objects()

    # Init. Methods
    
    def load_events_objects(self):
        self.events = ['Kettle', 'Tap', 'Oven', 'Fridge', 'DinnerwareCabinet', 'DrinkwareCabinet', 'FoodCabinet', 'CutleryDrawer', 'Bin', 'WashingMachine']
        self.objects = ['Bottle', 'WineGlass', 'Cup', 'Bowl', 'Plate', 'Fork', 'Knife', 'Spoon']
        self.classes = ['PreparingDrink', 'Cooking', 'WashingDishes', 'Cleaning', 'Tidying']

        self.init_objects()

    def init_objects(self):
        for obj in self.objects:
            predicate = 'involves_object(A,' + obj + ')'
            self.db << predicate
            self.db[predicate] = 0.0

    # ROS Methods

    def loop(self):
        while(True):
            while not rospy.core.is_shutdown():
                self.decay()
                self.ros_publish()
                rospy.rostime.wallsleep(0.5)
            
    def ros_evidence_callback(self, msg):
        print('[ROS] Received evidence:', msg.evidence, msg.etype)
        if msg.cmd == 'add':
            self.add(msg.evidence, msg.etype)
        elif msg.cmd == 'delete':
            self.delete(msg.evidence, msg.etype)
        else:
            print('[ROS] Invalid command in ROS evidence topic.')
            
    def ros_reason_callback(self, goal):
        pred, conf = self.reason()
        
        self._result.pred = pred
        self._result.conf = conf
        
        self._as.set_succeeded(self._result)
        
    def ros_reset_callback(self, msg):
        if msg.reset == 'reset':
            print('[ROS] Received reset command.')
            self.reset()

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

    # API Methods
    
    def reset(self):
        del self.db # unsafe behaviour, subscriber callback may write to db between these two function calls
        self.db = Database(self.mln)
        self.load_events_objects()
        print('[API', 'Reset.')

    def evidence(self):
        e_preds = []
        e_confs = []

        for e in self.db:
            e_preds.append(e[0])
            e_confs.append(e[1])

        # print('[API]', e_preds, e_confs)
        return e_preds, e_confs

    def reason(self):
        result = MLNQuery(mln=self.mln, db=self.db, method='EnumerationAsk').run()

        queries = []
        for cla in self.classes:
            predicate = 'class(A,' + cla + ')'
            queries.append(predicate)

        probs = []
        for q in queries:
            prob = result.results[q]
            probs.append(prob)
        
        winner = np.argmax(probs)
        
        print('[API]', self.classes[winner], probs[winner])
        return self.classes[winner], probs[winner]

    def add(self, evidence, etype):
        resp = 'OK'

        if etype == 'event':
            if evidence in self.events:
                predicate = 'involves_event(A,' + evidence + ')'
                self.db << predicate
            else:
                resp = 'Evidence not modelled in MLN.'
        elif etype == 'object':
            if evidence in self.objects:
                predicate = 'involves_object(A,' +evidence + ')'
                self.db << predicate
            else:
                resp = 'Evidence not modelled in MLN.'
        else:
            resp = 'Invalid evidence type. Valid types are: event or object'

        print('[API]', resp)
        return resp

    def delete(self, evidence, etype):
        resp = 'OK'

        # simple model of deletion, instantly removes, need to implement
        # (i) instant deletion for events
        # (ii) time-based deletion for objects (e.g. in 60 seconds rectractall will be called for predicate, use queue)
        if etype  == 'event':
            if evidence in self.events:
                predicate = 'involves_event(A,' + evidence + ')'
                self.db.retract(predicate)
            else:
                resp = 'Evidence not modelled in MLN.'
        elif etype == 'object':
            if evidence in self.objects:
                predicate = 'involves_object(A,' + evidence + ')'
                self.db.retract(predicate)
            else:
                resp = 'Evidence not modelled in MLN.'
        else:
            resp = 'Invalid evidence type. Valid types are: event or object' 

        print('[API]', resp)
        return resp

    # Additional Logic

    def decay(self):
        # do something here to decay events, e.g.
        # (i) events when added are put into a dictionary with a timestamp
        # (ii) every tick, we compare the timestamp of events with current timestamp
        # (iii) if timestamp is older than x (used a tiered system), then call delete on that event
        return

if __name__ == '__main__':
    # threading.Thread(target=lambda: rospy.init_node('robot_har_mln', disable_signals=True)).start()

    m = Main()

    app = Flask(__name__)
    CORS(app)

    threading.Thread(target=lambda: m.loop()).start()

    @app.route('/reset', methods = ['POST'])
    def reset_handler():

        m.reset()

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
        status = "OK"

        return jsonify(status)

    app.run(host='0.0.0.0', port = 5010)
