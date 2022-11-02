#! /usr/bin/env python3

# standard libraries
import rospy
from time import sleep

# internal classes
from log import Log

# standard messages
from darknet_ros_msgs.msg import BoundingBoxes

# custom messages
# none

# constants and parameters
# none

class ObjectPool():
    def __init__(self):
        # set up logger
        self.id = 'object_pool'
        self.logger = Log(self.id)

        # set up ROS
        self.ros_sub_bounding_boxes = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.ros_callback_bounding_boxes)
        # self.ros_pub_object_pool

        # instance variables
        self.object_pool = set()

        # ready
        self.logger.log_great('Ready.')

    # core logic

    def reset(self):
        self.object_pool = set()

    def get_object_pool(self):
        return self.object_pool

    # callbacks

    def ros_callback_bounding_boxes(self, msg):
        for bounding_box in msg.bounding_boxes:
            self.object_pool.add(bounding_box.Class)

if __name__ == '__main__':
    op = ObjectPool()