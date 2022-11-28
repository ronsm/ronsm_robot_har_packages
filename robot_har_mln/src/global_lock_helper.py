#! /usr/bin/env python3

# standard libraries
import rospy

# internal classes
from log import Log

# standard messages
from std_msgs.msg import Bool

# custom messages
# none

# constants and parameters
# none

class GlobalLockHelper():
    def __init__(self):
        # set up logger
        self.id = 'global_lock_helper'
        self.logger = Log(self.id)
        
        # set up ROS
        self.ros_pub_global_lock = rospy.Publisher('/ronsm_global_lock', Bool, queue_size=10)

        # instance variables
        self.state = False

        # ready
        self.logger.log_great('Ready.')

    def lock(self):
        msg = Bool()
        msg.data = True
        self.ros_pub_global_lock.publish(msg)
        self.state = True

    def unlock(self):
        msg = Bool()
        msg.data = False
        self.ros_pub_global_lock.publish(msg)
        self.state = False

    def state(self):
        return self.state