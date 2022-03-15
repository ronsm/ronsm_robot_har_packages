#! /usr/bin/env python3

import rospy
import tf
import hsrb_interface
from hsrb_interface import Robot

from log import Log

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion

ralt_map = {
    'kitchen' : (0.086, 0.519, 1.2),
    'dining' : (1.116, -1.593, 0.7),
    'lounge' : (1.840, -2.359, 3.0),
    'bedroom' : (0.012, -3.513, 0.5),
    'bathroom' : (-2.467, -1.727, 0)
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
        self.geometry = self.robot.try_get('geometry')

        self.logger.log_great('Ready.')

        rospy.spin()

    def ros_move_to_room_callback(self, msg):
        room = msg.data

        try:
            log = 'Moving to: ' + room
            self.logger.log(log)
            pos = ralt_map[room]
            self.move_to_pos(pos)
        except KeyError:
            self.logger.log_warn('Invalid room requested.')

    def move_to_pos(self, pos):
        x_pos = pos[0]
        y_pos = pos[1]
        theta = pos[2]

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position = Point(x_pos, y_pos, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        goal.pose.orientation = Quaternion(*quat)

        self.pub_pose.publish(goal)

        while(1):
            try:
                self.body.move_to_neutral()
                self.logger.log_great('Done moving.')
                break
            except hsrb_interface.exceptions.FollowTrajectoryError:
                self.logger.log('Still moving...')
                rospy.sleep(2)


if __name__ == '__main__':
    m = Main()