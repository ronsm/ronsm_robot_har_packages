#! /usr/bin/env python3

import rospy
import tf
import hsrb_interface
from hsrb_interface import Robot

from log import Log

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion

ralt_map = {
    'Kitchen' : (0.086, 0.519, 1.2),
    'Dining' : (1.116, -1.593, 0.7),
    'Lounge' : (1.840, -2.359, 4.9),
    'Bedroom' : (0.012, -3.513, 0.5),
    'Bathroom' : (-2.467, -1.727, 0)
}

class Main():
    def __init__(self):
        self.id = 'robot_har_robot_mover'

        self.logger = Log(self.id)

        rospy.init_node('robot_har_robot_mover')

        self.sub_move_to_room = rospy.Subscriber('/robot_har_robot_mover/move_to_room', String, callback=self.ros_move_to_room_callback)
        self.pub_pose = rospy.Publisher('/goal', PoseStamped, queue_size=10)

        self.robot = Robot()
        self.base = self.robot.try_get('omni_base')
        self.body = self.robot.try_get('whole_body')
        # self.geometry = self.robot.try_get('geometry')latest