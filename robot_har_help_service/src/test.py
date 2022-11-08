#!/usr/bin/env python3
import rospy
from speak import Speak
from hsrb_interface import Robot, exceptions

from move_to_room import MoveToRoom

from std_msgs.msg import String
from ronsm_messages.msg import dm_intent

# rospy.init_node('test2')

rospy.init_node('test')

speak = Speak()

action_end = False

def callback_action_end(msg):
    print('action end message')
    global action_end
    action_end = True

ros_pub_intent_bus = rospy.Publisher('/robot_har_rasa_core/intent_bus', dm_intent, queue_size=10)
ros_sub_action_end = rospy.Subscriber('/robot_har_rasa_core/action_end', String, callback=callback_action_end)

# actions

msg = dm_intent()
msg.intent = 'intent_pick_up_object'
msg.args = ['cup']
ros_pub_intent_bus.publish(msg)

while not action_end:
    rospy.sleep(1)
    print('waiting for action...')
action_end = False

rospy.sleep(2)

msg = dm_intent()
msg.intent = 'intent_go_to_room'
msg.args = ['bedroom']
ros_pub_intent_bus.publish(msg)

while not action_end:
    rospy.sleep(1)
    print('waiting for action...')
action_end = False

msg = dm_intent()
msg.intent = 'intent_hand_over'
msg.args = []
ros_pub_intent_bus.publish(msg)

# speak.request('Shall I take this to the bedroom?')
# speak.request('Are you ready to take the cup from me?')
# speak.request('Are you ready now?')
