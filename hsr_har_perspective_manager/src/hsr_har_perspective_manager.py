#!/usr/bin/env python3

import rospy
import actionlib
import threading

from time import sleep

from ralt_map_helper import RALTMapHelper
from openhab_helper import OpenHABHelper
from map_math import MapMath

from leg_tracker.msg import Person, PersonArray, Leg, LegArray
from geometry_msgs.msg import PointStamped, Point, Pose, Pose2D, PoseStamped, Quaternion
from face_detector.msg import FaceDetectorAction, FaceDetectorGoal
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from std_msgs.msg import Bool

import tf.transformations

class HSRHARPerspectiveManager():
    def __init__(self):
        # helper classes
        self.rmh = RALTMapHelper()
        self.ohh = OpenHABHelper('https://caregrouphwu%40icloud.com:G00drobot6@home.myopenhab.org:443/rest/items')
        threading.Thread(target=(lambda: self.ohh.spin())).start()
        self.mm = MapMath()

        # control flow
        self.run = False
        self.state = 0

        # important data
        self.person_array = []
        self.pose_received = False
        self.human_pose = None
        self.room = None
        self.rot_count = 0

        # ROS setup
        rospy.init_node('hsr_har_perspective_manager')
        self.rate = rospy.Rate(1)

        # ROS subscribers, publishers, action servers, and service handles
        rospy.Subscriber('/people_tracked', PersonArray, self.leg_tracker_callback, queue_size=10)
        rospy.Subscriber('/global_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/hsr_har_perspective_manager/control', Bool, self.control_callback, queue_size=10)

        self.mbas = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.mbas.wait_for_server()

        self.get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

# Control Flow

    """ 
    input: n/a
    ouput: n/a

    Implements a Finite State Machine (FSM) control flow.
    """
    def contol(self):
        while(True) and not rospy.is_shutdown():
            if self.state == 0:   # idle state
                if self.run:
                    self.state = 1
                else:
                    self.rate.sleep()
            elif self.state == 1: # check legs
                legs, self.human_pose = self.check_legs()
                if legs:
                    self.state = 2
                else:
                    self.state = 3
            elif self.state == 2: # nav to legs
                success = self.nav_to_viewpoint()
                if success:
                    self.state = 4
                else:
                    print('[WARN] Unable to navigate successfully to viewpoint.')
                    self.state = 4
            elif self.state == 3: # check OpenHAB
                success, self.room = self.check_openhab_location()
                if success:
                    self.state = 5
                else:
                    print('[WARN] Unable to detect room from OpenHAB. Will now search all rooms...')
                    self.state = 6
            elif self.state == 4: # check for faces
                success = self.check_faces()
                if success:
                    self.state = 7
                else:
                    self.state = 8
            elif self.state == 5: # navigate to room
                success = self.nav_to_room(self.room)
                if success:
                    self.state = 1
                else:
                    self.state = 6
            elif self.state == 6: # search every room for legs
                # TODO: search all rooms
                self.state = -1
            elif self.state == 7: # finished cycle
                print('[INFO] Arrived at viewpoint. Now idling...')
                self.state = 0
            elif self.state == 8: # rotate around human to get better view
                if self.rot_count <= 3:
                    success = self.rotate_around_human(self.human_pose, 45, reverse=False)
                elif self.rot_count >= 3 and self.rot_count <= 8:
                    success = self.rotate_around_human(self.human_pose, 45, reverse=True)
                else:
                    success = False
                    self.rot_count = 0
                    self.state = -1
                if success:
                    self.state = 4
                else:
                    print('[WARN] Unable to reach ideal new viewpoint. Trying another pose...')
                    self.state = 8
                self.rot_count = self.rot_count + 1
            elif self.state == -1: # error state
                print('[ERROR] Entered error state. Returning to idle...')
                self.state = 0
            else:                  # invalid state
                print('[ERROR] Entered an invalid state. Returning to idle...')
                self.state = 0


# ROS Callbacks

    """ 
    input: msg
    ouput: n/a

    Callback for the /hsr_har_perspective_manager/control topic, which starts/stops the package.
    """
    def control_callback(self, msg):
        if msg.data == True:
            self.run = True
        else:
            self.run = False

    """ 
    input: msg
    ouput: n/a

    Callback for the /pose topic, saving the pose to a class variable.
    """
    def pose_callback(self, msg):
        self.pose_received = True
        self.pose = msg

    """ 
    input: msg
    ouput: n/a

    Callback for the leg tracker topic /people_tracked, saving the data to a class variable.
    """
    def leg_tracker_callback(self, msg):
        self.person_array = msg

# ROS Action Servers

    """ 
    input: n/a
    ouput: faces detected

    Calls the face detector action server to ask for face detection.
    """
    def face_detector_client(self):
        client = actionlib.SimpleActionClient('face_detector_action', FaceDetectorAction)
        client.wait_for_server(timeout=rospy.Duration(10))
        goal = FaceDetectorGoal()
        client.send_goal(goal)
        client.wait_for_result(timeout=rospy.Duration(10))
        # TODO: Process the 
        return client.get_result()

    """ 
    input: target pose of the robot (target_pose)
    ouput: success (true/false)

    Interacts with /move_base/goal action server to move the robot to a new position.
    """
    def move_base_client(self, target_pose):
        print('[INFO] Begin move to:', target_pose[0], target_pose[1], target_pose[2])
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(target_pose[0], target_pose[1], 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, target_pose[2])
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose

        self.mbas.send_goal(goal)

        self.mbas.wait_for_result()

        action_state = self.mbas.get_state()

        if action_state == GoalStatus.SUCCEEDED:
            print('[INFO] Successfully moved to goal.')
            return True
        else:
            print('[INFO] Failed to move to goal.')
            return False

# ROS Service Calls

    """ 
    input: target pose of the robot (target_pose)
    ouput: plan

    Uses the /move_base/get_plan service to determine whether a target pose is reachable.
    This does not currently work on the HSR, since it does not enable the required service!
    """
    def check_point_validity(self, target_pose):
        req = GetPlan()
        req.start = self.pose

        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time(0)
        goal.pose.position.x = target_pose[0]
        goal.pose.position.y = target_pose[1]

        req.goal = goal

        req.tolerance = 0.5
        
        resp = self.get_plan(req.start, req.goal, req.tolerance)
        print(resp)

# Internal Logic

    """ 
    input: n/a
    ouput: whether there are legs (true/false), and the pose of the person if yes (pos)

    Checks if there are legs in the data received from the leg detector via the callback.
    If there are legs, select one as the target and return pose of the person.
    """
    def check_legs(self):
        # TODO: logic to check whether there are legs detected
        # PersonArray
        #     people[]
        #         id
        #         pose
        #             position
        #                 x
        #                 y
        #                 z
        #             orientation
        #                 x
        #                 y
        #                 z
        #                 w
        # msg.people[i].id
        # msg.people[i].pose.position.x
        if self.person_array:
            return True, pose
        else:
            return False, None

    """ 
    input: n/a
    ouput: whether a room can be inferred from OpenHAB (true/false), and if yes which room (room)

    Uses the OpenHAB helper to check which room the last sensor event occurred in.
    """
    def check_openhab_location(self):
        room = self.ohh.get_room()
        if room != 'wait':
            return True, room
        else:
            print('[DEBUG] No room has yet been detected by the OpenHAB helper.')
            return False, None

    """ 
    input: target pose of the robot (target_pose)
    ouput: whether the movement was successful (true/false)

    Uses the move_base client to move closer to the detected person.
    """
    def nav_to_viewpoint(self, target_pose):
        # TODO: given leg point nearby, calculate distance to legs, drive to 1.5 dist
        # use the move base client
        pass

    """ 
    input: name of the desired room to move to (room)
    ouput: whether the movement to the new room was successful 

    Uses the RALT map helper to find out the default pose for each room, and uses the move_base client to try and go there.
    """
    def nav_to_room(self, room):
        # TODO: get points of room from RALT helper
        #       use the move_base client to move there
        pass

    """ 
    input: pose of the detected human (human_pose), angle to turn (angle), and direction of rotation (reverse)
    ouput: whether the movement was successful (true/false)

    Calculates the new pose of the robot to move around the by a number of degrees and uses the move_base client to try and go there.
    """
    def rotate_around_human(self, human_pose, angle, reverse=False):
        robot_pose = [self.pose.position.x, self.pose.position.y, self.pose.orientation.z]
        
        human_pose[2] = self.mm.human_pose_rot(robot_pose)

        if reverse:
            new_robot_pose = self.mm.new_point_on_circumference(human_pose, 1.5, -angle, human_pose[2])
        else:
            new_robot_pose = self.mm.new_point_on_circumference(human_pose, 1.5, angle, human_pose[2])

        new_robot_pose[2] = self.mm.angle_to_face_point(new_robot_pose, human_pose)

        success = self.move_base_client(new_robot_pose)

        return success

    """ 
    input: n/a
    ouput: whether a face is detected (true/false)

    Uses the face detector action server to check whether faces are detected currently.
    """
    def check_faces(self):
        # TODO: call the face detector
        # look up toward face height
        # run the face detector
        pass

    """ 
    input: n/a
    ouput: n/a

    Waits for receipt of the pose message from the robot, confirmed by the /pose subscriber callback.
    """
    def wait_for_pose(self):
        while not self.pose_received:
            print('[DEBUG] Waiting for pose...')
            sleep(0.5)
        print('[DEBUG] Pose received.')

if __name__ == '__main__':
    hhpm = HSRHARPerspectiveManager()
    hhpm.wait_for_pose()
    hhpm.contol()