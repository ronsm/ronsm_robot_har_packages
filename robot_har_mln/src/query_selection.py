#! /usr/bin/env python3

import rospy

from log import Log

from ronsm_messages.msg import dm_system_request

class QuerySelection():
    def __init__(self):
        self.id = 'query_selection'

        self.logger = Log(self.id)

        self.pub_dm_request = rospy.Publisher('/robot_har_dialogue_system/system_request', dm_system_request, queue_size=10)

        self.logger.log_great('Ready.')

    def query_select(self, predictions_h, predictions_s):
        if len(predictions_s) < 3:
            return False, None
        else:
            if (predictions_s[-2][0] == predictions_s[-3][0]) and (predictions_s[-2][0] != predictions_s[-1][0]):
                return True, [predictions_s[-1][0], predictions_s[-2][0]]
            else:
                return False, None

    def label_query(self, options):
        msg = dm_system_request()
        msg.intent = 'har_adl_label_query'
        msg.args = options
        msg = 'Issuing query via dialogue system:' + str(options)
        self.logger.log_great(msg)
        self.pub_dm_request.publish(msg)