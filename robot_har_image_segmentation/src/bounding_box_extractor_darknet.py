#!/usr/bin/env python3
import rospy
import numpy as np
import pandas as pd
import ros_numpy
import pprint
import tf2_ros
import tf_conversions

MAX_WAIT = 10

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String, Int32
from sensor_msgs.msg import PointCloud2, Image
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox, ObjectCount

class BoundingBoxExtractorDarknet():
    def __init__(self):
        rospy.init_node('bounding_box_extractor_darknet')

        self.sub_req = rospy.Subscriber('/bounding_box_extractor_darknet/request', String, self.ros_callback_sub_req)
        self.sub_bbs = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.ros_callback_sub_bbs)
        self.sub_rgbd = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2, self.ros_callback_sub_rgbd)
        self.sub_rgbd_raw = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw', Image, self.ros_callback_sub_rgbd_raw)

        self.pub_depth = rospy.Publisher('/bounding_box_extractor/depth_out', PointCloud2, queue_size=10)
        self.pub_centre_depth = rospy.Publisher('/bounding_box_extractor/depth_centre_out', Int32, queue_size=10)

        self.pub_tf = tf2_ros.TransformBroadcaster()

        self.bbs = None
        self.depth = None
        self.image = None

        print('OK.')
        rospy.spin()

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

    def ros_callback_sub_bbs(self, msg):
        self.bbs = msg.bounding_boxes

    def ros_callback_sub_rgbd(self, msg):
        data = ros_numpy.numpify(msg)
        self.depth = data

    def ros_callback_sub_rgbd_raw(self, msg):
        data = ros_numpy.numpify(msg)
        self.image = data

    def segment(self, bb):
        count = 0
        while(self.depth == None):
            print('Waiting for depth image...')
            rospy.sleep(1.0)
            count = count + 1
            if count == MAX_WAIT:
                print('No depth image received.')
                break

        subsample = self.depth[bb.ymin:bb.ymax, bb.xmin:bb.xmax]
        print('Depth shape:', self.depth.shape)
        print('Subsample shape:', subsample.shape)

        msg = ros_numpy.msgify(PointCloud2, subsample)

        self.pub_depth.publish(msg)

        flat_subsample = subsample.flatten()
        sumd = 0
        for sample in flat_subsample:
            sumd = sumd + sample[2]
        avg = sumd / len(flat_subsample)
        x_pos = subsample.shape[1] / 2
        y_pos = subsample.shape[0] / 2
        print(subsample.shape)
        x_pos = int(round(x_pos, 0))
        y_pos = int(round(y_pos, 0))
        print(x_pos, y_pos)
        point = subsample[x_pos, y_pos]
        print('Centre:', point, avg)

        t = TransformStamped()

        print(point)

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'head_rgbd_sensor_link'
        t.child_frame_id = 'object_of_interest'
        t.transform.translation.x = point[0]
        t.transform.translation.y = point[1]
        t.transform.translation.z = point[2]
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        for i in range(0, 30):
            self.pub_tf.sendTransform(t)
            rospy.sleep(1)

if __name__ == '__main__':
    bbed = BoundingBoxExtractorDarknet()