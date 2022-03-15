#! /usr/bin/env python3

import rospy
import rospkg
import tf as tf1
import tf2_ros
import math
from hsrb_interface import Robot

from std_msgs.msg import String
from std_srvs.srv import Empty
from tmc_msgs.msg import Voice

marker_prefix = 'ar_marker'
look_at_marker = 'ar_marker/4000'

class Main():
    def __init__(self):
        rospy.init_node('robot_har_marker_utility')

        self.tf1 = tf1.TransformListener()

        self.tfBuffer = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.tfBuffer)

        self.sub_register_marker = rospy.Subscriber('robot_har_marker_utility/register_marker', String, callback=self.register_marker_callback)
        self.sub_look_at_marker = rospy.Subscriber('/robot_har_marker_utility/look_at_marker', String, callback=self.look_at_marker_callback)

        self.tts_pub = rospy.Publisher('/talk_request', Voice, queue_size=10)

        print('Waiting for AR marker services...')
        rospy.wait_for_service('/marker/start_recognition')
        rospy.wait_for_service('/marker/stop_recognition')
        print('AR marker services online. Awaiting request...')

        self.robot = Robot()
        self.base = self.robot.try_get('omni_base')
        self.whole_body = self.robot.try_get('whole_body')

        self.marker_recognition_enabled = False
        self.target_marker = None

        print('Ready.')

        rospy.spin()

    def enable_marker_recognition(self):
        try:
            set_recognition_mode = rospy.ServiceProxy('/marker/start_recognition', Empty)
            resp = set_recognition_mode()
            print('Started marker recognition.')
            self.marker_recognition_enabled = True
        except rospy.ServiceException as e:
                print('Service call failed to /marker/start_recognition')

    def disable_marker_recognition(self):
        try:
            set_recognition_mode = rospy.ServiceProxy('/marker/stop_recognition', Empty)
            resp = set_recognition_mode()
            print('Stopped marker recognition.')
            self.marker_recognition_enabled = False
        except rospy.ServiceException as e:
                print('Service call failed to /marker/stop_recognition')

    def register_marker_callback(self, msg):
        if not self.marker_recognition_enabled:
            self.enable_marker_recognition()

        self.say('Ok, I will try to register your marker.')

        rospy.sleep(2.0)

        self.whole_body.move_to_neutral()
        
        markers = []
        marker = False
        for i in range(0, 10):
            fs = self.tf1.getFrameStrings()
            markers = [s for s in fs if marker_prefix in s]
            print('Found markers:', markers)
            if len(markers) > 0:
                marker = True
                break
            rospy.sleep(1)

        if marker:
            self.target_marker = markers[0]
            print('Targetting marker:', self.target_marker)
            self.say('Ok, I will look at this marker.')
        else:
            self.target_marker = None
            print('Unable to detect marker.')
            self.say('Sorry, I was unable to detect a marker. If you like to try again, repeat the command.')        

    def look_at_marker_callback(self, msg):
       # self.whole_body.move_to_neutral()

        if not self.marker_recognition_enabled:
            self.enable_marker_recognition()

        rospy.sleep(0.5)

        marker = False
        if self.tfBuffer.can_transform('map', self.target_marker, rospy.Time(), timeout=rospy.Duration(10.0)):
            marker = True
            try:
                tf = self.tf1.lookupTransform('map', self.target_marker, rospy.Time())
                print(tf)
            except:
                print('Unable to lookup transform of marker.')

            angle = tf[1][2] + (math.pi / 2)
            opp_angle = (angle + math.pi) % (2 * math.pi)
            new_x = 1.0 * math.cos(opp_angle) + tf[0][0]
            new_y = 1.0 * math.sin(opp_angle) + tf[0][1]
            print(new_x, new_y, angle)

            self.base.go_abs(new_x, new_y, angle)

            self.whole_body.move_to_neutral()
        else:
            print('Unable to lookup transform of marker.')

        if not marker:
            print('Unable to locate marker.')
            self.say('Sorry, I was unable to locate the target marker. Please ensure the marker is visible and try again.')

    def say(self, say):
        msg = Voice()
        msg.language = 1
        msg.interrupting = True
        msg.queueing = True
        msg.sentence = say
        self.tts_pub.publish(msg)

if __name__ == '__main__':
    m = Main()