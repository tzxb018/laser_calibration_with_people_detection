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
from numpy.linalg import inv
import message_filters

hog_matrix = numpy.array([[]])
hog_pc = []
hog_pc_header = []
mouse_matrix = numpy.array([[]])
mouse_pc = []
mouse_pc_header = []

def callback_hog_matrx(data):
    global hog_matrix
    basis = numpy.array([[data.matrix_tf[0], data.matrix_tf[2]], [data.matrix_tf[1], data.matrix_tf[3]]])
    hog_matrix = basis

def callback_hog_pc(data):
    global hog_pc, hog_pc_header

    hog_pc_header = data.header
    hog_pc = data.points

def callback_mouse_matrix(data):
    global mouse_matrix
    basis = numpy.array([[data.matrix_tf[0], data.matrix_tf[2]], [data.matrix_tf[1], data.matrix_tf[3]]])
    mouse_matrix = basis
    # print(mouse_matrix)

def callback_mouse_pc(data):
    global mouse_pc, mouse_pc_header

    mouse_pc_header = data.header
    mouse_pc = data.points

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('matrix_listener', anonymous=True)

    hog_sub = rospy.Subscriber("/matrix_for_hog", matrix_tf, callback_hog_matrx)
    hog_bg_sub = rospy.Subscriber('/hog/bg_cloud', PointCloud, callback_hog_pc)
    hog_pub = rospy.Publisher('/pc_hog', PointCloud, queue_size=10)

    mouse_sub = rospy.Subscriber("/matrix_for_mouse", matrix_tf, callback_mouse_matrix)
    mouse_bg_sub = rospy.Subscriber('/mouse/bg_cloud', PointCloud, callback_mouse_pc)
    mouse_pub = rospy.Publisher('/pc_mouse', PointCloud, queue_size=10)


    while not rospy.is_shutdown():
        # print(hog_matrix)
        # print(hog_pc)

        hog_new_pc = PointCloud()
        hog_new_pc.header = copy.deepcopy(hog_pc_header)
        hog_new_pc.channels.append(ChannelFloat32())
        hog_new_pc.channels[0].name = "intensity"

        for pt in hog_pc:
            point_in_matrix_form = numpy.array([[pt.x],[pt.y]])
            # print(point_in_matrix_form)
            # print(hog_matrix)

            hog_new_point = numpy.matmul(hog_matrix, point_in_matrix_form)
            hog_new_pc.points.append(Point(hog_new_point[0][0], hog_new_point[1][0], 0.0))
            hog_new_pc.channels[0].values.append(.99)

        hog_pub.publish(hog_new_pc)


        mouse_new_pc = PointCloud()
        mouse_new_pc.header = copy.deepcopy(hog_pc_header)
        mouse_new_pc.channels.append(ChannelFloat32())
        mouse_new_pc.channels[0].name = "intensity"

        print(mouse_matrix)
        for pt in mouse_pc:
            point_in_matrix_form = numpy.array([[pt.x],[pt.y]])
            # print(point_in_matrix_form)
            # print(hog_matrix)

            mouse_new_point = numpy.matmul(mouse_matrix, point_in_matrix_form)
            mouse_new_pc.points.append(Point(mouse_new_point[0][0], mouse_new_point[1][0], 0.0))
            mouse_new_pc.channels[0].values.append(.99)

        mouse_pub.publish(mouse_new_pc)



if __name__ == '__main__':
    listener()
