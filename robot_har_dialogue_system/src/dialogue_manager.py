#!/usr/bin/env python3

import aiml
from log import Log
import responder
import rospy
import rospkg

from responder import Responder
from semantic_similarity import SemanticSimilarity
from semantic_ADLs import SemanticADLs
from input_output import InputOutput

from ronsm_messages.msg import har_arm_basic
from std_msgs.msg import String

MAX_TRIES = 3

class LabelEncapsulator(object):
    def __init__(self, model_label, ADL_label, semantic_description):
        self.model_label = model_label
        self.ADL_label = ADL_label
        self.semantic_description = semantic_description

class DialogueManager(object):
    def __init__(self, rel_path, label_linker):
        self.id = 'dialogue_manager'

        self.logger = Log(self.id)

        self.label_linker = label_linker

        self.rel_path = rel_path

        self.aiml = aiml.Kernel()
        aiml_path = self.rel_path + '/src/std-startup.xml'
        self.aiml.learn(aiml_path)
        self.aiml.respond('load aiml go')

        self.responder = Responder(self.rel_path)

        self.semantic_ADLs = SemanticADLs()
        self.labels_dict = self.semantic_ADLs.get_semantic_ADLs()

        self.semantic_similarity = SemanticSimilarity(self.semantic_ADLs)

        self.follow_up = False
        self.options = ['null', 'null']

        self.io = InputOutput(self.rel_path)

        self.pub_ros_arm_add_rule = rospy.Publisher('/robot_har_mln/arm/add_rule', har_arm_basic, queue_size=10)
        self.pub_ros_har_label = rospy.Publisher('/robot_har_dialogue_system/label', String, queue_size=10)

        self.logger.log_great('Ready.')
    
    def start_query(self, labels):
        reduced, query_type = self.process_labels(labels)

        if query_type == 1:
            self.story_query_1_label(reduced[0])
        elif query_type == 2:
            self.story_query_2_labels(reduced)
        elif query_type == 3:
            self.story_query_3_labels(reduced)
        else:
            self.logger.log_warn('A query process has been started when no query is required. Upstream error.')

    # Stories

    def story_query_1_label(self, confirmation_label):
        self.responder.query_1_label(confirmation_label)
        affirm_label = self.aiml.getPredicate('affirm_label')
        while affirm_label == '':
            res = self.get_input_and_respond()
            affirm_label = self.aiml.getPredicate('affirm_label')

        if affirm_label == 'true':
            self.responder.confirm_label(confirmation_label.semantic_description)
            # confirmation_label = self.label_linker.get_model_label(confirmation_label)

            log = 'Sending label to HAR system: ' + confirmation_label.model_label
            self.logger.log(log)
            msg = String()
            msg.data = confirmation_label.model_label
            self.pub_ros_har_label.publish(msg)

            self.aiml.setPredicate('affirm_label', 'false')
        elif affirm_label == 'false':
                self.story_query_all_labels()
        else:
            self.logger.log_warn('Invalid affirmation response. AIML error.')
            self.send_label_error()

        self.aiml.setPredicate('affirm_label', '')
        self.aiml.setPredicate('user_label', '')

    def story_query_2_labels(self, reduced):
        self.responder.query_2_labels(reduced)

        user_label = self.get_user_label_with_retries()

        follow_up, options, top_label, low_confidence = self.semantic_similarity.compare_similarity(user_label, labels=reduced)
        self.follow_up = follow_up
        self.options = options

        self.aiml.setPredicate('user_label', '')

        if follow_up or low_confidence:
            self.responder.query_2_labels_follow_up(options)

            user_label = self.get_user_label_with_retries()

            follow_up, options, top_label, low_confidence = self.semantic_similarity.compare_similarity(user_label, labels=options)

            if low_confidence:
                self.responder.low_confidence_label()
                self.follow_up = False
                self.aiml.setPredicate('user_label', '')
                self.send_label_error()
                return

        self.responder.confirm_label(user_label)
        user_label = self.label_linker.get_model_label(top_label)

        log = 'Sending label to HAR system: ' + user_label
        self.logger.log(log)
        msg = String()
        msg.data = user_label
        self.pub_ros_har_label.publish(msg)

        self.follow_up = False
        self.aiml.setPredicate('user_label', '')

    def story_query_3_labels(self, reduced):
        self.responder.query_3_labels()

        user_label = self.get_user_label_with_retries()

        follow_up, options, top_label, low_confidence = self.semantic_similarity.compare_similarity(user_label, labels=reduced)
        self.follow_up = follow_up
        self.options = options

        self.aiml.setPredicate('user_label', '')

        if follow_up or low_confidence:
            self.responder.query_2_labels_follow_up(options)

            user_label = self.get_user_label_with_retries()

            follow_up, options, top_label, low_confidence = self.semantic_similarity.compare_similarity(user_label, labels=options)

            if low_confidence:
                self.responder.low_confidence_label()
                self.follow_up = False
                self.aiml.setPredicate('user_label', '')
                return

        self.responder.confirm_label(user_label)
        user_label = self.label_linker.get_model_label(top_label)

        log = 'Sending label to HAR system: ' + user_label
        self.logger.log(log)
        msg = String()
        msg.data = user_label
        self.pub_ros_har_label.publish(msg)

        self.follow_up = False
        self.aiml.setPredicate('user_label', '')

    def story_query_all_labels(self):
        self.responder.query_3_labels()

        user_label = self.get_user_label_with_retries()

        follow_up, options, top_label, low_confidence = self.semantic_similarity.compare_similarity(user_label, compare_all=True)
        self.follow_up = follow_up
        self.options = options

        self.aiml.setPredicate('user_label', '')

        if follow_up or low_confidence:
            self.responder.query_2_labels_follow_up(options)

            user_label = self.get_user_label_with_retries()

            follow_up, options, top_label, low_confidence = self.semantic_similarity.compare_similarity(user_label, labels=options)

            if low_confidence:
                self.responder.low_confidence_label()
                self.follow_up = False
                self.aiml.setPredicate('user_label', '')
                return

        self.responder.confirm_label(user_label)
        user_label = self.label_linker.get_model_label(top_label)

        log = 'Sending label to HAR system: ' + user_label
        self.logger.log(log)
        msg = String()
        msg.data = user_label
        self.pub_ros_har_label.publish(msg)

        self.follow_up = False
        self.aiml.setPredicate('user_label', '')

    def story_query_all_labels_teaching(self):
        self.responder.query_3_labels_teaching()

        user_label = self.get_user_label_with_retries()

        follow_up, options, top_label, low_confidence = self.semantic_similarity.compare_similarity(user_label, compare_all=True)
        self.follow_up = follow_up
        self.options = options

        self.aiml.setPredicate('user_label', '')

        if follow_up or low_confidence:
            self.responder.query_2_labels_follow_up(options)

            user_label = self.get_user_label_with_retries()

            follow_up, options, top_label, low_confidence = self.semantic_similarity.compare_similarity(user_label, labels=options)

            if low_confidence:
                self.responder.low_confidence_label_teaching()
                self.follow_up = False
                self.aiml.setPredicate('user_label', '')
                return 'Other'

        self.responder.confirm_label(user_label)
        user_label = self.label_linker.get_model_label(top_label)

        log = 'Sending label to HAR system: ' + user_label
        self.logger.log(log)
        msg = String()
        msg.data = user_label
        self.pub_ros_har_label.publish(msg)

        self.follow_up = False
        self.aiml.setPredicate('user_label', '')
        
        return user_label

    def get_user_label_with_retries(self):
        user_label = self.aiml.getPredicate('user_label')
        tries = 0
        while user_label == '':
            res = self.get_input_and_respond()
            if not res and tries < MAX_TRIES:
                tries = tries + 1
                self.responder.sorry_please_try_again()
                continue
            user_label = self.aiml.getPredicate('user_label')

        return user_label

    # Non-Story Dynamic Responses

    def confirm_monitor_rule(self):
        when = self.aiml.getPredicate('when')
        do = self.aiml.getPredicate('do')

        follow_up, options, top_label, low_confidence = self.semantic_similarity.compare_similarity(when, [], compare_all=True)

        when_adl = top_label
        when_desc = self.semantic_ADLs.get_ADL_descriptor(top_label)

        self.responder.confirm_monitor_rule(when_desc, do)
    
        self.follow_up = False

        self.aiml.setPredicate('responder', '')
        self.aiml.setPredicate('when', '')
        self.aiml.setPredicate('do', '')

        msg = har_arm_basic()
        msg.when = when_adl
        msg.do = do
        self.pub_ros_arm_add_rule.publish(msg)

    # Tools

    def process_labels(self, labels):
        reduced = []
        count = 0

        for label in labels:
            if label not in reduced:
                count = count + 1
                reduced.append(label)
                
        label_encapsulators = []
        for label in reduced:
            model_label = label
            ADL_label = self.label_linker.get_ADL_labels(label)
            semantic_description = self.label_linker.get_model_label_description(label)
            
            le = LabelEncapsulator(model_label, ADL_label, semantic_description)

            label_encapsulators.append(le)

        return label_encapsulators, count

    def get_input_and_respond(self):
        input = self.io.listen()

        self.aiml.respond(input)
        method = self.aiml.getPredicate('responder')
        
        if method == 'bypass':
            return True
        elif method != '':
            handle = getattr(self.responder, method)
            handle()
            return True
        else:
            self.logger.log_warn('No valid response.')
            return False

    def respond_to_input(self, msg):
        self.aiml.respond(msg)
        method = self.aiml.getPredicate('responder')

        if method == 'bypass':
            return
        elif method != '':
            handle = getattr(self, method)
            handle()
        else:
            self.logger.log_warn('No valid response.')

    def send_label_error(self, msg):
        log = 'Sending label error message to HAR system.'
        self.logger.log(log)
        msg = String()
        msg.data = 'LABEL_ERROR'
        self.pub_ros_har_label.publish(msg)