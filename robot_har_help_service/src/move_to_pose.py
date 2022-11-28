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
# none

class MoveToPose():
    def __init__(self, body):
        # set up logger
        self.id = 'move_to_pose'
        self.logger = Log(self.id)

        # set up ROS
        self.ros_ac_move_base = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.ros_ac_move_base.wait_for_server()

        # set up HSR
        self.body = body

        # instance variables
        # none

        # ready
        self.logger.log_great('Ready.')

    def request(self, pose):
        log = 'Attempting to move to specified pose...'
        self.logger.log(log)

        self.body.move_to_neutral()

        pose.header.stamp = rospy.Time.now()
        
        goal = MoveBaseGoal()
        goal.target_pose = pose

        self.ros_ac_move_base.send_goal(goal)
        
        self.ros_ac_move_base.wait_for_result(rospy.Duration(60))

        action_state = self.ros_ac_move_base.get_state()
        if action_state == GoalStatus.SUCCEEDED:
            self.logger.log_great('Action completed successfully.')
            msg = String()
            self.pub_workspace_pose.publish(msg)
            return True
        else:
            self.logger.log_warn('Action failed to complete. Ensure path to location is not obstructed.')
            return False