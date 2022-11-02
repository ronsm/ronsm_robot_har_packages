#! /usr/bin/env python3

# standard libraries
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import sys

# internal classes
from log import Log

# standard messages
from std_msgs.msg import String
from tmc_msgs.msg import Voice, TalkRequestAction, TalkRequestGoal

# custom messages
# none

# constants and parameters
# none

class Speak():
    def __init__(self):
        # set up logger
        self.id = 'speak'
        self.logger = Log(self.id)

        # set up ROS
        self.ros_pub_tts = rospy.Publisher('/talk_request', Voice, queue_size=10)

        # set up HSr
        talk_as = '/talk_request_action'
        self.ros_as_talk = actionlib.SimpleActionClient(talk_as, TalkRequestAction)

        try:
            if not self.ros_as_talk.wait_for_server(rospy.Duration(20)):
                self.logger.log_warn('Talk action server could not be found.')
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

        # ready
        self.logger.log_great('Ready.')

    def request(self, text):
        goal = TalkRequestGoal()
        goal.data.language = Voice.kEnglish
        goal.data.sentence = text

        self.ros_as_talk.send_goal(goal)

        self.ros_as_talk.wait_for_result(timeout=rospy.Duration(40))

        rospy.sleep(1)

        self.logger.log(text)