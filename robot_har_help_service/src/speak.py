#! /usr/bin/env python3

# standard libraries
import rospy
# internal classes
from log import Log

# standard messages
from tmc_msgs.msg import Voice
from std_msgs.msg import String

# custom messages
# none

# constants and parameters
# none

class Speak():
    def __init__(self, speak):
        # set up logger
        self.id = 'speak'
        self.logger = Log(self.id)

        # set up ROS
        self.ros_pub_tts = rospy.Publisher('/talk_request', Voice, queue_size=10)

        # ready
        self.logger.log_great('Ready.')

    def request(self, text):
        log = 'Sending to HSR TTS: ' + text
        self.logger.log(log)

        msg = Voice()
        msg.language = 1
        msg.interrupting = True
        msg.queueing = True
        msg.sentence = text

        self.ros_pub_tts.publish(msg)