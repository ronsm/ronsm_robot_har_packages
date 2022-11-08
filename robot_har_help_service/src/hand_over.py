#! /usr/bin/env python3

# standard libraries
import rospy

# internal classes
from log import Log

# standard messages
# none

# custom messages
# none

# constants and parameters
# none

class HandOver():
    def __init__(self, speak, body, grip):
        # set up logger
        self.id = 'speak'
        self.logger = Log(self.id)

        # set up ROS
        # none

        # set up HSR
        self.body = body
        self.grip = grip
        self.speak = speak

        # ready
        self.logger.log_great('Ready.')

    # core logic

    def request(self):
        try:
            print('Handing over object...')
            self.body.move_to_joint_positions({'arm_flex_joint' : -0.6, 'arm_lift_joint' : 0.25, 'arm_roll_joint' : 0.5, 'wrist_flex_joint' : -1.0})
            self.speak.request('Here you go!')
            self.grip.command(1.0)
            rospy.sleep(5)
            self.body.move_to_neutral()
        except:
            self.logger.log_warn('Failed to open gripper.')