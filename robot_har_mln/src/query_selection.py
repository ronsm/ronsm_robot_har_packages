#! /usr/bin/env python3

import rospy

from log import Log

from ronsm_messages.msg import dm_system_request

class QuerySelection():
    def __init__(self):
        self.id = 'query_selection'

        self.logger = Log(self.id)

        # Label waiting
        self.waiting_for_response = False
        self.query_type = 0
        self.prediction_s = None
        self.prediction_s_f = None
        self.label_s2_if_matches = None

        self.pub_dm_request = rospy.Publisher('/robot_har_dialogue_system/system_request', dm_system_request, queue_size=10)

        self.logger.log_great('Ready.')

    def query_select(self, prediction_h, prediction_s, prediction_h_f, prediction_s_f, consistent_s, consistent_s_f, agree):
        cm = consistent_s
        cf = consistent_s_f
        ag = agree

        query = False
        query_type = None
        query_options = []
        label_s2_if_matches = False

        # combinational boolean logic table
        if (cm == True) and (cf == True) and (ag == True):
            query = True
            query_type = 1
            query_options = [prediction_s]
            label_s2_if_matches = False
    
        if (cm == False) and (cf == True) and (ag == True):
            query = True
            query_type = 2
            options = [prediction_s, prediction_s_f]
            label_s2_if_matches = True

        if (cm == True) and (cf == False) and (ag == True):
            query = True
            query_type = 2
            query_options = [prediction_s, prediction_s_f]
            label_s2_if_matches = True

        if (cm == False) and (cf == False) and (ag == True):
            query = True
            query_type = 3
            query_options = [prediction_s, prediction_s, prediction_s] # this is dummy input, DM expects 3 args.
            label_s2_if_matches = False

        if (cm == True) and (cf == True) and (ag == False):
            query = True
            query_type = 2
            query_options = [prediction_s, prediction_s_f]
            label_s2_if_matches = True

        if (cm == False) and (cf == True) and (ag == False):
            query = True
            query_type = 2
            query_options = [prediction_s, prediction_s_f]
            label_s2_if_matches = True

        if (cm == True) and (cf == False) and (ag == False):
            query = True
            query_type = 2
            query_options = [prediction_s, prediction_s_f]
            label_s2_if_matches = True

        if (cm == False) and (cf == False) and (ag == False):
            query = True
            query_type = 3
            query_options = [prediction_s, prediction_s, prediction_s] # this is dummy input, DM expects 3 args.
            label_s2_if_matches = False

        if query:
            log = 'Creating query (type, options, prediction_s, prediction_s_f): ' + str(query_type) + ', ' + str(query_options) + ', ' + str(prediction_s) + ', ' + str(prediction_s_f)
            self.logger.log(log)
            self.label_query(query_options)
            self.set_waiting_for_response(True, query_type, prediction_s, prediction_s_f, label_s2_if_matches)

    def set_waiting_for_response(self, yes_no, query_type=None, prediction_s=None, prediction_s_f=None, label_s2_if_matches=None):
        if not yes_no:
            self.waiting_for_response = False
            self.query_type = 0
            self.prediction_s = None
            self.prediction_s_f = None
            self.label_s2_if_matches = None
        else:
            self.waiting_for_response = True
            self.query_type = query_type
            self.prediction_s = prediction_s
            self.prediction_s_f = prediction_s_f
            self.label_s2_if_matches = label_s2_if_matches

    def is_waiting_for_label(self):
        return self.waiting_for_response

    def apply_to(self, label):
        if self.prediction_s_f == label:
            if self.label_s2_if_matches:
                return True
            else:
                return False
        else:
            return False

    def label_query(self, options):
        msg = dm_system_request()
        msg.intent = 'har_adl_label_query'
        msg.args = options
        log = 'Issuing query via dialogue system:' + str(options)
        self.logger.log_great(log)
        self.pub_dm_request.publish(msg)