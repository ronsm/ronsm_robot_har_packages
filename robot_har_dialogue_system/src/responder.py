#!/usr/bin/env python3
import numpy as np

from log import Log
from input_output import InputOutput

class Responder(object):
    def __init__(self):
        self.id = 'responder'

        self.logger = Log(self.id)

        self.io = InputOutput()

        self.logger.log_great('Ready.')

    # Say

    def say_hello(self):
        choices = []

        choices.append('Hello!')
        choices.append('Hi!')
        choices.append('Good day to you!')

        choice = np.random.choice(choices)
        
        self.io.say(choice)

    # HAR Query

    def query_1_label(self, confirmation_label):
        choices = []

        msg = 'Can you confirm that you are currently ' + confirmation_label.semantic_description + '?'
        choices.append(msg)
        msg = 'It looks to me like you are ' + confirmation_label.semantic_description + '. Can you confirm this for me?'
        choices.append(msg)

        choice = np.random.choice(choices)

        self.io.say(choice)

    def query_2_labels(self, labels):
        print(labels)
        choices = []
        
        msg = 'It looks like you are either ' + labels[0].semantic_description  + ' or ' + labels[1].semantic_description + '. Can you confirm for me which?'
        choices.append(msg)
        msg = 'It appears to me that you are currently ' + labels[0].semantic_description + ' or ' + labels[1].semantic_description + '. Can you tell me which is correct?'
        choices.append(msg)

        choice = np.random.choice(choices)

        self.io.say(choice)

    def query_3_labels(self):
        choices = []

        choices.append('I\'m not quite sure what you are doing at the moment. Can you tell me what you are doing?')
        choices.append('I need some help understanding what you are doing just now. Can you tell me what you are doing?')

        choice = np.random.choice(choices)

        self.io.say(choice)

    def query_2_labels_follow_up(self, options):
        choices = []

        msg = 'I\'m having a little trouble matching what you said to my activity labels, but I think I\'ve narrowed it down to two. Are you ' + options[0] + ' or ' + options[1] + '?'
        choices.append(msg)
        msg = 'Sorry, I\'ve not been able to match that up immediately, but I think it might be one of these two options. Can you confirm whether you are ' + options[0] + ' or ' + options[1] + '?'

        choice = np.random.choice(choices)

        self.io.say(choice)

    def confirm_label(self, label):
        choices = []

        msg = 'Ok, I have recorded that you are ' + label + '.'
        choices.append(msg)
        msg = 'I have noted that you are ' + label + '.'
        choices.append(msg)

        choice = np.random.choice(choices)

        self.io.say(choice)

    def low_confidence_label(self):
        msg = 'Sorry, I was unable to find a suitable matching label. I will not label this activity.'
        self.io.say(msg)

    def low_confidence_label_teaching(self):
        msg = 'Sorry, I was unable to find a suitable matching label. I will label this activity as other.'
        self.io.say(msg)

    def sorry_please_try_again(self):
        msg = 'Sorry, I didnt understand that. Please try again.'
        self.io.say(msg)

# User Intent Responses

    def start_teaching_adl(self):
        choices = []

        choices.append('Ok, I am ready to learn your activity. Begin when you are ready.')
        choices.append('Sure, I can learn your activity. Start whenever you are ready.')
        
        choice = np.random.choice(choices)

        self.io.say(choice)

    def stop_teaching_adl(self):
        choices = []

        msg = 'Ok, I am done learning your activity.'
        choices.append(msg)
        msg = 'Great, I have learned your activity.'
        choices.append(msg)
        
        choice = np.random.choice(choices)

        self.io.say(choice)

    def query_3_labels_teaching(self):
        choices = []

        choices.append('Can you confirm for me what this activity was?')
        choices.append('Just to make sure I have this right, can you tell me what it was you were just doing?')

        choice = np.random.choice(choices)

        self.io.say(choice)

    def confirm_monitor_rule(self, when, do):
        choices = []

        msg = 'Ok, I will ' + do + ' when they are ' + when
        choices.append(msg)
        msg = 'Sure, I can do that when they are ' + when
        choices.append(msg)

        choice = np.random.choice(choices)

        self.io.say(choice)