#!/usr/bin/env python3

# standard libraries
import rospy
from hsrb_interface import geometry
import actionlib
import tf

# internal classes
from log import Log
from check_preconditions import CheckPreconditions

# standard messages
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from std_msgs.msg import Bool

# custom messages
# none

# constants and parameters
# constants and parameters
GRASP_FORCE = 0.2
MARKER_TF = 'ar_marker/2000'
HAND_TF = 'hand_palm_link'
MARKER_LOCATION = (-0.1079, 1.1827, 0.75)

MARKER_TO_HAND = geometry.pose(z=-0.02, ek=-1.57)
HAND_UP = geometry.pose(x=0.1)
HAND_BACK = geometry.pose(z=-0.5)

class PickMarker():
    def __init__(self, speak, base, body, grip):
        # set up logger
        self.id = 'pick_marker'
        self.logger = Log(self.id)

        # set up ROS
        self.ros_pub_aligned = rospy.Publisher('/robot_har_help_service/marker_align/aligned', Bool, queue_size=10)

        self.ros_ac_move_base = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.ros_ac_move_base.wait_for_server()

        rospy.wait_for_service('/marker/start_recognition')
        rospy.wait_for_service('/marker/stop_recognition')

        # set up HSR
        self.speak = speak
        self.base = base
        self.body = body
        self.grip = grip

        # set up classes
        self.cp = CheckPreconditions()

        # instance variables
        self.marker_recognition_enabled = False

        # ready
        self.logger.log_great('Ready.')

    def request(self):
        success = True

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

        if success:
            try:
                self.logger.log('Opening gripper...')
                self.grip.command(1.0)
            except:
                self.logger.log_warn('Fail to initialize.')
                success = False

        if success:
            try:
                self.logger.log('Moving to where the marker can be seen...')
                result = self.move(MARKER_LOCATION[0], MARKER_LOCATION[1], MARKER_LOCATION[2])
                if not result:
                    success = True
            except:
                self.logger.log_warn('Failed to move to the position.')
                success = True

        if success:
            try:
                self.logger.log('Moving to neutral...')
                self.body.move_to_neutral()
                self.body.move_to_joint_positions({'arm_lift_joint' : 0.18})
            except:
                self.logger.log_warn('Unable to move to neutral.')
                success = False

        rospy.sleep(2.0)

        if success:
            self.logger.log('Waiting on precondition...')
            precondition_met = self.cp.wait_for_precondition('intent_pick_up_object', ['bottle'], 20)
            if not precondition_met:
                success = False

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

        if success:
            try:
                self.logger.log('Moving to neutral...')
                self.body.move_to_neutral()
            except:
                self.logger.log_warn('Unable to move to neutral.')
                success = False

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

    def move_to_workspace(self):
        success = True

        if success:
            try:
                self.logger.log('Moving to neutral...')
                self.body.move_to_neutral()
            except:
                self.logger.log_warn('Unable to move to neutral.')
                success = False

        try:
            self.logger.log('Moving to where the workspace position...')
            result = self.move(MARKER_LOCATION[0], MARKER_LOCATION[1], MARKER_LOCATION[2])
            if not result:
                success = True
        except:
            self.logger.log_warn('Failed to move to the position.')
            success = True

        if success:
            try:
                self.logger.log('Moving to neutral...')
                self.body.move_to_neutral()
            except:
                self.logger.log_warn('Unable to move to neutral.')
                success = False

        if success:
            msg = Bool()
            msg.data = True
            self.ros_pub_aligned.publish(msg)

        return success