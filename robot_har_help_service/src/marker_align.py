#! /usr/bin/env python3

# standard libraries
import rospy
import tf as tf1
import tf2_ros
import math

# internal classes
from log import Log

# standard messages
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
from tmc_msgs.msg import Voice

# custom messages
# none

# constants and parameters
MARKER_PREFIX = 'ar_marker'
MARKER_DEFAULT = 'ar_marker/6'

class MarkerAlign():
    def __init__(self, speak, base, body):
        # set up logger
        self.id = 'marker_align'
        self.logger = Log(self.id)

        # set up ROS
        self.tf1 = tf1.TransformListener()
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2 = tf2_ros.TransformListener(self.tf2_buffer)

        self.ros_pub_aligned = rospy.Publisher('/robot_har_help_service/marker_align/aligned', Bool, queue_size=10)

        rospy.wait_for_service('/marker/start_recognition')
        rospy.wait_for_service('/marker/stop_recognition')

        # set up HSR
        self.base = base
        self.body = body
        self.speak = speak

        # instance variables
        self.marker_recognition_enabled = False
        self.target_marker = MARKER_DEFAULT

        # ready
        self.logger.log_great('Ready.')

    # core logic

    def request_register_marker(self):
        if not self.marker_recognition_enabled:
            self.enable_marker_recognition()

        self.speak.request('Ok, I will try to register your marker.')

        rospy.sleep(2.0)
        
        markers = []
        marker = False
        for i in range(0, 10):
            fs = self.tf1.getFrameStrings()
            markers = [s for s in fs if MARKER_PREFIX in s]
            if len(markers) > 0:
                marker = True
                break
            rospy.sleep(1)

        if marker:
            self.target_marker = markers[0]
            log = 'Targetting marker: ' + self.target_marker
            self.logger.log(log)
            self.speak.request('Ok, I will look at this marker.')
            return True
        else:
            self.target_marker = None
            self.logger.log_warn('Unable to detect marker.')
            self.speak.request('Sorry, I was unable to detect a marker. If you like to try again, repeat the command.')    
            return False   

    def request_look_at_marker(self):
        if not self.marker_recognition_enabled:
            self.enable_marker_recognition()
            rospy.sleep(0.5)

        if not self.marker_recognition_enabled:
            return False

        self.speak.request('I will now try to align with your marker.')

        self.body.move_to_joint_positions({'arm_flex_joint': -2.5, 'wrist_flex_joint': 1.2})
        self.body.move_to_joint_positions({'head_tilt_joint': 0.0})
        self.body.move_to_joint_positions({'arm_lift_joint': 0.45})
        rospy.sleep(0.5)

        for i in range(0, 20):
            marker = False
            marker_tf = None
            if self.tfBuffer.can_transform('map', self.target_marker, rospy.Time(), timeout=rospy.Duration(10.0)):
                marker = True
                try:
                    marker_tf = self.tf1.lookupTransform('map', self.target_marker, rospy.Time())

                    robot_tf_to_map = self.tf1.lookupTransform('map', 'base_link', rospy.Time())
                    robot_euler_r, robot_euler_p, robot_euler_y = self.euler_from_quaternion(robot_tf_to_map[1][0], robot_tf_to_map[1][1], robot_tf_to_map[1][2], robot_tf_to_map[1][3])
                    # print('Robot Euler:', robot_euler_r, robot_euler_p, robot_euler_y)

                    marker_tf_to_base_link = self.tf1.lookupTransform('base_link', self.target_marker, rospy.Time())
                    marker_euler_r, marker_euler_p, marker_euler_y = self.euler_from_quaternion(marker_tf_to_base_link[1][0], marker_tf_to_base_link[1][1], marker_tf_to_base_link[1][2], marker_tf_to_base_link[1][3])
                    # print('Marker Euler:', marker_euler_r, marker_euler_p, marker_euler_y)
                except:
                    self.logger.log_warn('Unable to lookup transform of marker.')

            if marker:
                new_x = marker_tf[0][0]
                new_y = marker_tf[0][1]
                marker_euler_y = marker_euler_y + (math.pi / 2)
                new_angle = robot_euler_y + marker_euler_y
                new_angle = new_angle % (2 * math.pi)
                # print('Marker Euler:', marker_euler_y, 'New Angle:', new_angle)

                try:
                    self.base.go_abs(new_x, new_y, new_angle)
                    pass
                except:
                    self.speak.request('Ok, that is as close as I can get to the marker.')
                    msg = Bool()
                    msg.data = True
                    self.ros_pub_aligned.publish(msg)
                    self.body.move_to_joint_positions({'head_tilt_joint': 0.0})
                    break
            else:
                self.logger.log_warn('Unable to lookup transform of marker.')

            self.body.move_to_joint_positions({'head_tilt_joint': 0.0})

        rospy.sleep(0.1)

        if not marker:
            self.logger.log_warn('Unable to locate the target marker.')
            self.speak.request('Sorry, I was unable to locate the target marker. Please ensure the marker is visible and try again.')

        if self.marker_recognition_enabled:
            self.disable_marker_recognition()

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

    # service calls

    def enable_marker_recognition(self):
        try:
            set_recognition_mode = rospy.ServiceProxy('/marker/start_recognition', Empty)
            resp = set_recognition_mode()
            self.logger.log('Enabled marker recognition.')
            self.marker_recognition_enabled = True
        except rospy.ServiceException as e:
                self.logger.log_warn('Service call failed to /marker/start_recognition.')

    def disable_marker_recognition(self):
        try:
            set_recognition_mode = rospy.ServiceProxy('/marker/stop_recognition', Empty)
            resp = set_recognition_mode()
            self.logger.log('Disabled marker recognition.')
            self.marker_recognition_enabled = False
        except rospy.ServiceException as e:
                self.logger.log_warn('Service call failed to /marker/stop_recognition.')