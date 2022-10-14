#!/usr/bin/env python3

import sys
import rospy
import rospkg
import requests

from log import Log
from input_output import InputOutput

RASA_WEBHOOK = 'http://localhost:5005/webhooks/rest/webhook'
OUTPUT = 'ROBOT'
INPUT = 'KEYBOARD'

class Main():
    def __init__(self):
        self.id = 'main'
        self.logger = Log(self.id)
        self.logger.startup_msg()

        rospack = rospkg.RosPack()
        self.rel_path = rospack.get_path('robot_har_rasa')

        self.io = InputOutput(self.rel_path, OUTPUT)

        rospy.init_node('robot_har_rasa')

        self.logger.log_great('Ready.')

        self.spin()

    # Spin

    def spin(self):
        while not rospy.is_shutdown():
            cmd = input('~>')
            if cmd == 'exit':
                sys.exit(0)
            elif cmd == 'x':
                if OUTPUT == 'KEYBOARD':
                    utterance = input('utterance: ')
                elif OUTPUT == 'MICROPHONE':
                    utterance = self.io.listen() # enable this to get input from microphone instead of keyboard
            self.send_to_rasa(utterance)
            rospy.sleep(0.5)

    # RASA Interaction

    def send_to_rasa(self, utterance):
        request = {'message': utterance}

        response = requests.post(RASA_WEBHOOK, json = request)

        response = response.json()

        response = response[0]

        message = response['text']

        print(message)
        self.io.say(message)

if __name__ == '__main__':
    m = Main()