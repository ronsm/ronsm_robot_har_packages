#!/usr/bin/env python3

# standard libraries
import hsrb_interface
import rospy
import requests
import tf.transformations
import sys
from hsrb_interface import geometry

# internal classes
from log import Log

# standard messages
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from tmc_msgs.msg import Voice

# custom messages
# none

# constants and parameters
GRASP_FORCE = 0.2
OBJECT_TF = 'object_of_interest'
HAND_TF = 'hand_palm_link'

OBJECT_TO_HAND = geometry.pose(z=-0.02, ek=-1.57)
HAND_UP = geometry.pose(x=0.1)
HAND_BACK = geometry.pose(z=-0.5)

class PickFromTF():
    def __init__(self, speak, base, body, grip):
        # set up logger
        self.id = 'pick_from_tf'
        self.logger = Log(self.id)

        # set up HSR
        self.speak = speak
        self.base = base
        self.body = body
        self.grip = grip

        # ready
        self.logger.log_great('Ready.')

    def request(self):
        error = False

        if not error:
            try:
                self.logger.log('Opening gripper...')
                self.grip.command(1.0)
            except:
                error = True
                self.logger.log_warn('Failed to open gripper.')

        if not error:
            try:
                self.logger.log('Trying to grasp...')

                self.body.looking_hand_constraint = True
                self.body.move_end_effector_pose(OBJECT_TO_HAND, OBJECT_TF)
                
                self.grip.apply_force(GRASP_FORCE)

                rospy.sleep(0.5)

                self.body.move_end_effector_pose(HAND_UP, HAND_TF)
                self.body.move_end_effector_pose(HAND_BACK, HAND_TF)

                self.body.move_to_neutral()
            except Exception as e:
                print(e)
                error = True
                self.logger.log_warn('Failed to grasp.')

        if error:
            self.logger.log_warn('An error occured while executing this action.')

        if error:
            return False
        else:
            return True