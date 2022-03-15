#!/usr/bin/env python3

import queue
import rospy

from log import Log

from std_msgs.msg import String

class HARInterface():
    def __init__(self):
        self.id = 'har_interface'
        self.logger = Log(self.id)

        self.pub_new_rule_start = rospy.Publisher('/robot_har_mln/mln/new_rule_start', String, queue_size=10)
        self.pub_new_rule_stop = rospy.Publisher('/robot_har_mln/mln/new_rule_stop', String, queue_size=10)
        self.pub_new_rule_label = rospy.Publisher('/robot_har_mln/mln/label', String, queue_size=10)
        self.pub_register_marker = rospy.Publisher('/robot_har_marker_utility/register_marker', String, queue_size=10)
        self.pub_look_at_marker = rospy.Publisher('/robot_har_marker_utility/look_at_marker', String, queue_size=10)

        self.logger.log_great('Ready.')

    def start_teaching_adl(self):
        msg = String()
        msg.data = 'start'

        self.pub_new_rule_start.publish(msg)

    def stop_teaching_adl(self):
        msg = String()
        msg.data = 'stop'

        self.pub_new_rule_stop.publish(msg)

    def label_teaching_adl(self, label):
        msg = String()
        msg.data = label
        
        self.pub_new_rule_label.publish(msg)

    def register_marker(self):
        msg = String()
        msg.data = ''
        
        self.pub_register_marker.publish(msg)

    def look_at_marker(self):
        msg = String()
        msg.data = ''

        self.pub_look_at_marker.publish(msg)