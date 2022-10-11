#!/usr/bin/env python3
import rospy
import numpy as np
import pandas as pd
import ros_numpy

from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image, PointCloud2
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox, ObjectCount

MAX_WAIT = 10

class BoundingBoxExtractor():
    def __init__(self):
        rospy.init_node('bounding_box_extractor')

        self.sub_req = rospy.Subscriber('/bounding_box_extractor/request', String, self.ros_callback_sub_req)
        self.sub_bbs = rospy.Subscriber('/bounding_box_extractor/bbs_in', BoundingBoxes, self.ros_callback_sub_bbs)
        self.sub_rgbd = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2, self.ros_callback_sub_rgbd)

        self.pub_depth = rospy.Publisher('/bounding_box_extractor/depth_out', PointCloud2, queue_size=10)
        self.pub_centre_depth = rospy.Publisher('/bounding_box_extractor/depth_centre_out', Int32, queue_size=10)

        self.bbs = None
        self.depth = None

        rospy.spin()

    # ROS Callbacks

    def ros_callback_sub_bbs(self, msg):
        self.bbs = msg.bounding_boxes

    def ros_callback_sub_rgbd(self, msg):
        data = ros_numpy.numpify(msg)
        self.depth = data

    def ros_callback_sub_req(self, msg):
        count = 0
        while(self.bbs == None):
            print('Waiting for bounding boxes...')
            rospy.sleep(1.0)
            count = count + 1
            if count == MAX_WAIT:
                print('No bounding boxes received.')
                break

        if self.bbs != None:
            for bb in self.bbs:
                if bb.Class == msg.data:
                    self.segment(bb)

    # Logic

    def segment(self, bb):
        count = 0
        while(self.depth == None):
            print('Waiting for depth image...')
            rospy.sleep(1.0)
            count = count + 1
            if count == MAX_WAIT:
                print('No depth image received.')
                break

        subsample = self.depth[bb.xmin:bb.xmax, bb.ymin:bb.ymax]

        msg = ros_numpy.msgify(PointCloud2, subsample)

        self.pub_depth.publish(msg)

bbe = BoundingBoxExtractor()