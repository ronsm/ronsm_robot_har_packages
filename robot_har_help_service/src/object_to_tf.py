#!/usr/bin/env python3

# standard libraries
import rospy
import ros_numpy
import tf2_ros
import tf_conversions

# internal classes
from log import Log

# standard messages
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
from sensor_msgs.msg import PointCloud2, Image
from darknet_ros_msgs.msg import BoundingBoxes

# custom messages
# none

# constants and parameters
MAX_WAIT = 30

class ObjectToTF():
    def __init__(self, speak):
        # set up logger
        self.id = 'object_to_tf'
        self.logger = Log(self.id)

        # set up ROS
        self.ros_sub_bounding_boxes = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.ros_callback_bounding_boxes)
        self.ros_sub_rgbd = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2, self.ros_callback_rgbd)
        self.ros_sub_rgbd_raw = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw', Image, self.ros_callback_rgbd_raw)

        self.ros_pub_depth = rospy.Publisher('/robot_har_help_service/object_to_tf/depth_out', PointCloud2, queue_size=10)
        self.ros_pub_centre_depth = rospy.Publisher('/robot_har_help_service/object_to_tf/depth_centre_out', Int32, queue_size=10)
        self.ros_pub_tf = tf2_ros.StaticTransformBroadcaster()

        # set up HSR
        self.speak = speak

        # instance variables
        self.bounding_boxes = None
        self.depth = None
        self.image = None

        # ready
        self.logger.log_great('Ready.')

    # core logic

    def request(self, target):
        waits = 0
        while(self.bounding_boxes == None):
            self.logger.log('Waiting for bounding boxes...')
            rospy.sleep(1.0)
            waits = waits + 1
            if waits == MAX_WAIT:
                self.logger.log_warn('No bounding boxes were received on time. Check that DarknetROS is running and the objects are visible.')
                return False

        waits = 0
        while(self.depth == None):
            self.logger.log('Waiting for depth image...')
            rospy.sleep(1.0)
            waits = waits + 1
            if waits == MAX_WAIT:
                self.logger.log_warn('No depth images were received on time. Ensure that the robot is not under unusually high CPU load, or increase wait time.')
                return False

        for bounding_box in self.bounding_boxes:
            if bounding_box.Class == target:
                self.segment(bounding_box)
                return True

        return False

    def segment(self, bb):
        subsample = self.depth[bb.ymin:bb.ymax, bb.xmin:bb.xmax]
        print('Depth shape:', self.depth.shape)
        print('Subsample shape:', subsample.shape)

        msg = ros_numpy.msgify(PointCloud2, subsample)

        self.ros_pub_depth.publish(msg)

        flat_subsample = subsample.flatten()
        sumd = 0
        for sample in flat_subsample:
            sumd = sumd + sample[2]
        avg = sumd / len(flat_subsample)
        row = subsample.shape[0] / 2
        col = subsample.shape[1] / 2
        row = int(round(row, 0))
        col = int(round(col, 0))
        print(row, col)
        point = subsample[row, col]
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

        self.ros_pub_tf.sendTransform(t)

    # callbacks

    def ros_callback_bounding_boxes(self, msg):
        self.bounding_boxes = msg.bounding_boxes

    def ros_callback_rgbd(self, msg):
        data = ros_numpy.numpify(msg)
        self.depth = data

    def ros_callback_rgbd_raw(self, msg):
        data = ros_numpy.numpify(msg)
        self.image = data