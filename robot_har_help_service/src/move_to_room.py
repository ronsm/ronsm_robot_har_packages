#! /usr/bin/env python3

# standard libraries
from turtle import position
import rospy
import actionlib
import tf

# internal classes
from log import Log

# standard messages
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# custom messages
# none

# constants and parameters
RALT_MAP = {
    'kitchen' : (0.086, 0.519, 1.2),
    'dining' : (1.116, -1.593, 0.7),
    'lounge' : (1.840, -2.359, 4.9),
    'bedroom' : (-1.87, -3.97, 4.8),
    'bathroom' : (-2.467, -1.727, 0)
}

class MoveToRoom():
    def __init__(self, body):
        # set up logger
        self.id = 'move_to_room'
        self.logger = Log(self.id)

        # set up ROS
        self.ros_sub_move_to_room = rospy.Subscriber('/robot_har_help_service/move_to_room/request', String, callback=self.ros_callback_move_to_room)
        self.ros_sub_workspace_pose = rospy.Subscriber('/robot_har_help_service/move_to_room/workspace_pose', String, callback=self.ros_callback_workspace_pose)
        self.ros_ac_move_base = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.ros_ac_move_base.wait_for_server()

        # set up HSR
        self.body = body

        # instance variables
        self.current_room = None

        # ready
        self.logger.log_great('Ready.')

    # core logic

    def request(self, room):
        if room == self.current_room:
            return

        log = 'Attempting to move to:' + room
        self.logger.log(log)

        self.body.move_to_neutral()

        destination = RALT_MAP[room]

        goal_x = destination[0]
        goal_y = destination[1]
        goal_r = destination[2]

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(goal_x, goal_y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, goal_r)
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose

        self.ros_ac_move_base.send_goal(goal)

        self.ros_ac_move_base.wait_for_result(rospy.Duration(60))

        action_state = self.ros_ac_move_base.get_state()
        if action_state == GoalStatus.SUCCEEDED:
            self.logger.log_great('Action completed successfully.')
            return True
        else:
            self.logger.log_warn('Action failed to complete. Ensure path to location is not obstructed.')
            return False

    # callbacks

    def ros_callback_move_to_room(self, msg):
        room = msg.data
        room = room.lower()
        self.request(room)

    def ros_callback_workspace_pose(self, msg):
        self.body.move_to_joint_positions({'head_tilt_joint': 0.0, 'arm_lift_joint' : 0.25})