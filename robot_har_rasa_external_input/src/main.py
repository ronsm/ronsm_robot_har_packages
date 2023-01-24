#!/usr/bin/env python3

# standard libraries
import rospy

# internal classes
# none

# standard messages
from std_msgs.msg import String

# custom messages
# none

# constants and parameters
# none

class Main():
    def __init__(self):
        rospy.init_node('robot_har_rasa_external_input')

        self.ros_pub_external_input = rospy.Publisher('/robot_har_rasa/external_input', String, queue_size=10)

        while(1):
            utterance = input('~> ')
            if utterance != '':
                msg = String()
                msg.data = utterance
                self.ros_pub_external_input.publish(msg)

if __name__ == '__main__':
    m = Main()