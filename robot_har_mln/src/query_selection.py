#! /usr/bin/env python3

from subprocess import call
import rospy

from log import Log

from ronsm_messages.msg import dm_al_request

class QuerySelection():
    def __init__(self):
        self.id = 'query_selection'

        self.logger = Log(self.id)

        # Label waiting
        self.waiting_for_response = False
        self.query_type = 0
        self.prediction_s1 = None
        self.prediction_s2 = None
        self.label_s2_if_matches = None

        self.pub_dm_request = rospy.Publisher('/robot_har_dialogue_system/al_request', dm_al_request, queue_size=10)

        self.logger.log_great('Ready.')

    def query_select(self, prediction_s1, prediction_s2, consistent_s1, consistent_s2, agree):
        cs1 = consistent_s1
        cs2 = consistent_s2
        ag = agree

        query = False
        query_type = None
        query_options = []
        label_s2_if_matches = False

        # combinational boolean logic table
        if (cs1 == True) and (cs2 == True) and (ag == True):
            query = True
            query_type = 1
            query_options = [prediction_s1]
            label_s2_if_matches = False
    
        if (cs1 == False) and (cs2 == True) and (ag == True):
            query = True
            query_type = 2
            options = [prediction_s1, prediction_s2]
            label_s2_if_matches = True

        if (cs1 == True) and (cs2 == False) and (ag == True):
            query = True
            query_type = 2
            query_options = [prediction_s1, prediction_s2]
            label_s2_if_matches = True

        if (cs1 == False) and (cs2 == False) and (ag == True):
            query = True
            query_type = 3
            query_options = [prediction_s1, prediction_s1, prediction_s1] # this is dummy input, DM expects 3 args.
            label_s2_if_matches = False

        if (cs1 == True) and (cs2 == True) and (ag == False):
            query = True
            query_type = 2
            query_options = [prediction_s1, prediction_s2]
            label_s2_if_matches = True

        if (cs1 == False) and (cs2 == True) and (ag == False):
            query = True
            query_type = 2
            query_options = [prediction_s1, prediction_s2]
            label_s2_if_matches = True

        if (cs1 == True) and (cs2 == False) and (ag == False):
            query = True
            query_type = 2
            query_options = [prediction_s1, prediction_s2]
            label_s2_if_matches = True

        if (cs1 == False) and (cs2 == False) and (ag == False):
            query = True
            query_type = 3
            query_options = [prediction_s1, prediction_s1, prediction_s1] # this is dummy input, DM expects 3 args.
            label_s2_if_matches = False

        if query:
            log = 'Creating query (type, options, prediction_s1, prediction_s2): ' + str(query_type) + ', ' + str(query_options) + ', ' + str(prediction_s1) + ', ' + str(prediction_s2)
            self.logger.log(log)
            self.label_query(query_options)
            self.set_waiting_for_response(True, query_type, prediction_s1, prediction_s2, label_s2_if_matches)

    def set_waiting_for_response(self, yes_no, query_type=None, prediction_s1=None, prediction_s2=None, label_s2_if_matches=None):
        if not yes_no:
            self.waiting_for_response = False
            self.query_type = 0
            self.prediction_s1 = None
            self.prediction_s2 = None
            self.label_s2_if_matches = None
        else:
            self.waiting_for_response = True
            self.query_type = query_type
            self.prediction_s1 = prediction_s1
            self.prediction_s2 = prediction_s2
            self.label_s2_if_matches = label_s2_if_matches

    def is_waiting_for_label(self):
        return self.waiting_for_response

    def cancel_query(self):
        self.set_waiting_for_response(False)

    def apply_to(self, label):
        if self.prediction_s2 == label:
            if self.label_s2_if_matches:
                return True
            else:
                return False
        else:
            return False

    def label_query(self, options):
        msg = dm_al_request()
        msg.args = options
        log = 'Issuing query via dialogue system:' + str(options)
        self.logger.log_great(log)
        self.pub_dm_request.publish(msg)