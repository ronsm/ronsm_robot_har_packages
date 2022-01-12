#!/usr/bin/env python3

import aiml
from log import Log
import responder

from responder import Responder
from semantic_similarity import SemanticSimilarity
from semantic_ADLs import SemanticADLs
from input_output import InputOutput

class LabelEncapsulator(object):
    def __init__(self, model_label, ADL_label, semantic_description):
        self.model_label = model_label
        self.ADL_label = ADL_label
        self.semantic_description = semantic_description

class DialogueManager(object):
    def __init__(self, label_linker):
        self.id = 'dialogue_manager'

        self.logger = Log(self.id)

        self.label_linker = label_linker

        self.aiml = aiml.Kernel()
        self.aiml.learn('std-startup.xml')
        self.aiml.respond('load aiml go')

        self.responder = Responder()

        self.semantic_ADLs = SemanticADLs()
        self.labels_dict = self.semantic_ADLs.get_semantic_ADLs()

        self.semantic_similarity = SemanticSimilarity(self.semantic_ADLs)

        self.follow_up = False
        self.options = ['null', 'null']

        self.io = InputOutput()

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
            self.get_input_and_respond()
            affirm_label = self.aiml.getPredicate('affirm_label')

        if affirm_label == 'true':
            self.responder.confirm_label(confirmation_label)
            confirmation_label = self.label_linker.get_model_label(confirmation_label)
            print(confirmation_label)
        elif affirm_label == 'false':
                self.story_query_all_labels()
        else:
            self.logger.log_warn('Invalid affirmation response. AIML error.')

        self.aiml.setPredicate('user_label', '')

    def story_query_2_labels(self, reduced):
        self.responder.query_2_labels(reduced)

        user_label = self.aiml.getPredicate('user_label')
        while user_label == '':
            self.get_input_and_respond()
            user_label = self.aiml.getPredicate('user_label')

        follow_up, options, top_label = self.semantic_similarity.compare_similarity(user_label, labels=reduced)
        self.follow_up = follow_up
        self.options = options

        self.aiml.setPredicate('user_label', '')

        if follow_up:
            self.responder.query_2_labels_follow_up(options)

            user_label = self.aiml.getPredicate('user_label')
            while user_label == '':
                self.get_input_and_respond()
                user_label = self.aiml.getPredicate('user_label')

            follow_up, options, top_label = self.semantic_similarity.compare_similarity(user_label, labels=options)

        self.responder.confirm_label(user_label)
        user_label = self.label_linker.get_model_label(top_label)
        print(user_label)

        self.follow_up = False
        self.aiml.setPredicate('user_label', '')

    def story_query_3_labels(self, reduced):
        self.responder.query_3_labels()

        user_label = self.aiml.getPredicate('user_label')
        while user_label == '':
            self.get_input_and_respond()
            user_label = self.aiml.getPredicate('user_label')

        follow_up, options, top_label = self.semantic_similarity.compare_similarity(user_label, labels=reduced)
        self.follow_up = follow_up
        self.options = options

        self.aiml.setPredicate('user_label', '')

        if follow_up:
            self.responder.query_2_labels_follow_up(options)

            user_label = self.aiml.getPredicate('user_label')
            while user_label == '':
                self.get_input_and_respond()
                user_label = self.aiml.getPredicate('user_label')

            follow_up, options, top_label = self.semantic_similarity.compare_similarity(user_label, labels=options)

        self.responder.confirm_label(user_label)
        user_label = self.label_linker.get_model_label(top_label)
        print(user_label)

        self.follow_up = False
        self.aiml.setPredicate('user_label', '')

    def story_query_all_labels(self):
        self.responder.query_3_labels()

        user_label = self.aiml.getPredicate('user_label')
        while user_label == '':
            self.get_input_and_respond()
            user_label = self.aiml.getPredicate('user_label')

        follow_up, options, top_label = self.semantic_similarity.compare_similarity(user_label, compare_all=True)
        self.follow_up = follow_up
        self.options = options

        self.aiml.setPredicate('user_label', '')

        if follow_up:
            self.responder.query_2_labels_follow_up(options)

            user_label = self.aiml.getPredicate('user_label')
            while user_label == '':
                self.get_input_and_respond()
                user_label = self.aiml.getPredicate('user_label')

            follow_up, options, top_label = self.semantic_similarity.compare_similarity(user_label, labels=options)

        self.responder.confirm_label(user_label)
        user_label = self.label_linker.get_model_label(top_label)
        print(user_label)

        self.follow_up = False
        self.aiml.setPredicate('user_label', '')

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
            return
        elif method != '':
            handle = getattr(self.responder, method)
            handle()
        else:
            self.logger.log_warn('No valid response.')