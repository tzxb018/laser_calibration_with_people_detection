#!/usr/bin/env python
import rospy
import math
import copy
import sys
# from rtree import index
import matplotlib.pyplot as pp
import tf
import laser_geometry as lp
import numpy
from sensor_msgs.msg import LaserScan, PointCloud2, PointCloud, ChannelFloat32
import sensor_msgs.point_cloud2 as pc
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA, Float64MultiArray
from lc.msg import matrix_tf
import random


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('matrix_listener', anonymous=True)

    hog_sub = rospy.Subscriber("matrix_for_hog", matrix_tf, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
