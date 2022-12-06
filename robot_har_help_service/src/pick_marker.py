#!/usr/bin/env python3

# standard libraries
import rospy
from hsrb_interface import geometry
import actionlib

# internal classes
from log import Log

# standard messages
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

# custom messages
# none

# constants and parameters
# constants and parameters
GRASP_FORCE = 0.2
MARKER_TF = 'ar_marker/2000'
HAND_TF = 'hand_palm_link'
MARKER_LOCATION = (-0.1079, 1.1827, -0.0485)

MARKER_TO_HAND = geometry.pose(z=-0.02, ek=-1.57)
HAND_UP = geometry.pose(x=0.1)
HAND_BACK = geometry.pose(z=-0.5)

class PickMarker():
    def __init__(self, speak, base, body, grip):
        # set up logger
        self.id = 'pick_marker'
        self.logger = Log(self.id)

        # set up ROS
        self.ros_ac_move_base = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.ros_ac_move_base.wait_for_server()

        rospy.wait_for_service('/marker/start_recognition')
        rospy.wait_for_service('/marker/stop_recognition')

        # set up HSR
        self.speak = speak
        self.base = base
        self.body = body
        self.grip = grip

        # instance variables
        self.marker_recognition_enabled = False

        # ready
        self.logger.log_great('Ready.')

    def request(self):
        success = True
        stage = 0

        if not self.marker_recognition_enabled:
            self.enable_marker_recognition()
            rospy.sleep(0.5)

        if not self.marker_recognition_enabled:
            success = False

        if success:
            try:
                self.logger.log('Moving to neutral...')
                self.body.move_to_neutral()
            except:
                self.logger.log_warn('Unable to move to neutral.')
                success = False
                stage = 0

        if success:
            try:
                self.logger.log('Opening gripper...')
                self.grip.command(1.0)
            except:
                self.logger.log_warn('Fail to initialize.')
                success = False
                stage = 1

        # if success:
        #     try:
        #         self.logger.log('Moving to where the marker can be seen...')
        #         result = self.move(MARKER_LOCATION[0], MARKER_LOCATION[1], MARKER_LOCATION[2])
        #         if not result:
        #             success = True
        #             stage = 2
        #     except:
        #         self.logger.log_warn('Failed to move to the position.')
        #         success = True
        #         stage = 2

        if success:
            try:
                self.logger.log('Trying to pick...')

                self.body.looking_hand_constraint = True
                self.body.move_end_effector_pose(MARKER_TO_HAND, MARKER_TF)

                self.grip.apply_force(GRASP_FORCE)

                rospy.sleep(0.5)

                self.body.move_end_effector_pose(HAND_UP, HAND_TF)
                self.body.move_end_effector_pose(HAND_BACK, HAND_TF)

                self.body.move_to_neutral()
            except:
                self.logger.log_warn('Failed to grasp.')
                success = False
                stage = 3

        if self.marker_recognition_enabled:
            self.disable_marker_recognition()

        return success
       
    def move(self, x, y, theta):
        log = 'Attempting to move to specified pose...'
        self.logger.log(log)

        self.body.move_to_neutral()
        
        goal = MoveBaseGoal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position = Point(x, y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation = Quaternion(*quat)

        self.ros_ac_move_base.send_goal(goal)
        
        self.ros_ac_move_base.wait_for_result(rospy.Duration(60))

        action_state = self.ros_ac_move_base.get_state()
        if action_state == GoalStatus.SUCCEEDED:
            self.logger.log_great('Action completed successfully.')
            return True
        else:
            self.logger.log_warn('Action failed to complete. Ensure path to location is not obstructed.')
            return False

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
