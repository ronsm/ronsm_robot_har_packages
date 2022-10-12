from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher

import rospy
from std_msgs.msg import String

rospy.init_node('robot_har_rasa_core')

pub_intent_bus = rospy.Publisher('/robot_har_rasa_core/intent_bus', String, queue_size=10)

class ActionSimpleROSCommand(Action):

    def name(self) -> Text:
        return 'action_simple_ros_command'

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        intent = tracker.get_intent_of_latest_message()

        msg = String()
        msg.data = str(intent)

        pub_intent_bus.publish(msg)

        return []