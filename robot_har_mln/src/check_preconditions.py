#!/usr/bin/env python3

# standard libraries
import rospy

# internal classes
from log import Log

# standard messages
from std_msgs.msg import String, Bool
from darknet_ros_msgs.msg import BoundingBoxes

# custom messages
# none

# constants and parameters
WAIT_FOR_BBS = 10

class CheckPreconditions():
    def __init__(self):
        # set up logger
        self.id = 'check_preconditions'
        self.logger = Log(self.id)

        # set up ROS
        self.ros_sub_bounding_boxes = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.ros_callback_bounding_boxes)

        # instance variables
        self.bounding_boxes = None

        # ready
        self.logger.log_great('Ready.')

    # core logic

    def wait_for_precondition(self, intent, args, max_wait):
        success = False

        if intent == 'intent_pick_up_object':
            success = self.wait_for_object(args[0], max_wait)
        else:
            success = True

        return success

    def wait_for_object(self, target, max_wait):
        waits = 0
        while(self.bounding_boxes is None):
            self.logger.log('Waiting for bounding boxes...')
            rospy.sleep(1.0)
            waits = waits + 1
            if waits == WAIT_FOR_BBS:
                self.logger.log_warn('No bounding boxes were received on time. Check that DarknetROS is running and the objects are visible.')
                return False

        found = False
        waits = 0
        while (waits < max_wait):
            self.logger.log('Waiting for precondition...')
            for bounding_box in self.bounding_boxes:
                if bounding_box.Class == target:
                    print(bounding_box)
                    if (bounding_box.xmin > 220) and (bounding_box.xmax < 410):
                        self.logger.log('Yes')
                        found = True
            waits = waits + 1
            if found:
                break
            rospy.sleep(1.0)

        if found:
            log = 'Detected precondition object: ' + target
            self.logger.log(log)
        else:
            log = 'Unable to detect precondition object: ' + target
            self.logger.log_warn(log)

        return found

    # callbacks

    def ros_callback_detect_flask_request(self, msg):
        self.request()

    def ros_callback_bounding_boxes(self, msg):
        self.bounding_boxes = msg.bounding_boxes