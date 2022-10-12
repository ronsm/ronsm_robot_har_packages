#! /usr/bin/env python3

import rospy
import rospkg
import tf as tf1
import tf2_ros
import math
from hsrb_interface import Robot

from log import Log

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose2D, PoseStamped
from std_srvs.srv import Empty
from tmc_msgs.msg import Voice

marker_prefix = 'ar_marker'
look_at_marker = 'ar_marker/6'

class Main():
    def __init__(self):
        self.id = 'main'
        self.logger = Log(self.id)
        self.logger.startup_msg()

        rospy.init_node('robot_har_marker_utility')

        # Transform Listeners
        self.tf1 = tf1.TransformListener()
        self.tfBuffer = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.tfBuffer)

        # ROS Subscribers
        self.sub_entity_bus = rospy.Subscriber('/robot_har_rasa_core/entity_bus', String, callback=self.callback_entity_bus)

        # ROS Publishers
        self.pub_aligned = rospy.Publisher('/robot_har_marker_utility/aligned', Bool, queue_size=10)
        self.pub_tts = rospy.Publisher('/talk_request', Voice, queue_size=10)

        # Marker Recognition Services
        print('Waiting for AR marker services...')
        rospy.wait_for_service('/marker/start_recognition')
        rospy.wait_for_service('/marker/stop_recognition')
        print('AR marker services online. Awaiting request...')

        # HSR API
        self.robot = Robot()
        self.base = self.robot.try_get('omni_base')
        self.whole_body = self.robot.try_get('whole_body')

        # Class Variables
        self.marker_recognition_enabled = False
        self.target_marker = look_at_marker

        self.logger.log_great('Ready.')
        
        rospy.spin()

    # Enable/Disable Marker Recognition

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

    # Core Functionality

    def register_marker(self):
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
            self.logger.log_warn('Unable to detect marker.')
            self.say('Sorry, I was unable to detect a marker. If you like to try again, repeat the command.')        

    def look_at_marker(self):
        # enable marker recognition
        if not self.marker_recognition_enabled:
            self.enable_marker_recognition()
            rospy.sleep(0.5)

        # if unable to enable marker recognition, do not continue
        if not self.marker_recognition_enabled:
            rospy.logerr('Unable to enable marker recognition.')
            return

        self.say('I will now try to align with your marker.')

        self.whole_body.move_to_joint_positions({'arm_flex_joint': -2.5, 'wrist_flex_joint': 1.2})
        self.whole_body.move_to_joint_positions({'head_tilt_joint': 0.0})
        self.whole_body.move_to_joint_positions({'arm_lift_joint': 0.45})
        rospy.sleep(0.5)

        # find the marker
        for i in range(0, 20):
            marker = False
            marker_tf = None
            if self.tfBuffer.can_transform('map', self.target_marker, rospy.Time(), timeout=rospy.Duration(10.0)):
                marker = True
                try:
                    marker_tf = self.tf1.lookupTransform('map', self.target_marker, rospy.Time())

                    robot_tf_to_map = self.tf1.lookupTransform('map', 'base_link', rospy.Time())
                    robot_euler_r, robot_euler_p, robot_euler_y = self.euler_from_quaternion(robot_tf_to_map[1][0], robot_tf_to_map[1][1], robot_tf_to_map[1][2], robot_tf_to_map[1][3])
                    print('Robot Euler:', robot_euler_r, robot_euler_p, robot_euler_y)

                    marker_tf_to_base_link = self.tf1.lookupTransform('base_link', self.target_marker, rospy.Time())
                    marker_euler_r, marker_euler_p, marker_euler_y = self.euler_from_quaternion(marker_tf_to_base_link[1][0], marker_tf_to_base_link[1][1], marker_tf_to_base_link[1][2], marker_tf_to_base_link[1][3])
                    print('Marker Euler:', marker_euler_r, marker_euler_p, marker_euler_y)
                except:
                    self.logger.log_warn('Unable to lookup transform of marker.')

            if marker:
                new_x = marker_tf[0][0]
                new_y = marker_tf[0][1]
                marker_euler_y = marker_euler_y + (math.pi / 2)
                new_angle = robot_euler_y + marker_euler_y
                new_angle = new_angle % (2 * math.pi)
                print('Marker Euler:', marker_euler_y, 'New Angle:', new_angle)

                try:
                    self.base.go_abs(new_x, new_y, new_angle)
                    pass
                except:
                    self.say('Ok, that is as close as I can get to the marker.')
                    msg = Bool()
                    msg.data = True
                    self.pub_aligned.publish(msg)
                    self.whole_body.move_to_joint_positions({'head_tilt_joint': 0.0})
                    break
            else:
                self.logger.log_warn('Unable to lookup transform of marker.')

            self.whole_body.move_to_joint_positions({'head_tilt_joint': 0.0})

        rospy.sleep(0.1)

        if not marker:
            print('Unable to locate marker.')
            self.say('Sorry, I was unable to locate the target marker. Please ensure the marker is visible and try again.')
        
        # disable marker recognition
        if self.marker_recognition_enabled:
            self.disable_marker_recognition()

    # ROS Callback
    def callback_entity_bus(self, msg):
        intent = msg.data

        if intent == 'intent_register_marker':
            self.register_marker()
        elif intent == 'intent_look_at_marker':
            self.look_at_marker()
        else:
            self.logger.log_warn('Invalid intent. Disregarding input.')

    # HSR TTS

    def say(self, say):
        msg = Voice()
        msg.language = 1
        msg.interrupting = True
        msg.queueing = True
        msg.sentence = say
        self.pub_tts.publish(msg)

    # Math Helper Functions

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians

if __name__ == '__main__':
    m = Main()