#! /usr/bin/env python3

# standard libraries
import rospy
from hsrb_interface import collision_world, geometry

# internal classes
from log import Log

# standard messages
# none

# custom messages
# none

# constants and parameters
# none

class CollisionArray():
    def __init__(self, colw):
        # set up logger
        self.id = 'collision_array'
        self.logger = Log(self.id)

        # set up ROS
        # none

        # set up HSR
        self.colw = colw

        # instance variables
        # none

        self.logger.log('Adding collision objects...')
        self.colw.remove_all()
        # worktop_left = self.colw.add_box(x=1.9, y=0.6, z=0.9, pose=geometry.pose(x=-1.0, y=2.9, z=0.45, ek=0.70), frame_id='map', name='worktop_left')
        # worktop_right = self.colw.add_box(x=3.0, y=0.6, z=0.9, pose=geometry.pose(x=0.6, y=2.0, z=0.45, ek=2.275), frame_id='map', name='worktop_right')

        # ready
        self.logger.log_great('Ready.')