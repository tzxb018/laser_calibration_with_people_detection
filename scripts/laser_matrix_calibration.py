#!/usr/bin/env python
import rospy
import math
import copy
import sys
import sympy
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
from lc.srv import Laser_tf

hog_matrix = numpy.array([[]])
hog_pc = []
hog_pc_header = []
hog_ref_point_c = (0, 0)
hog_ref_point_a = (0, 0)
mouse_matrix = numpy.array([[]])
mouse_pc = []
mouse_pc_header = []
mouse_ref_point_c = (0,0)
mouse_ref_point_a = (0,0)

def callback_hog_matrx(data):
    global hog_matrix, hog_ref_point_c, hog_ref_point_a
    basis = numpy.array([[data.matrix_tf[0], data.matrix_tf[2]], [data.matrix_tf[1], data.matrix_tf[3]]])
    hog_matrix = basis
    hog_ref_point_c = (data.matrix_tf[4], data.matrix_tf[5])
    hog_ref_point_a = (data.matrix_tf[6], data.matrix_tf[7])

def callback_hog_pc(data):
    global hog_pc, hog_pc_header

    hog_pc_header = data.header
    hog_pc = data.points

def callback_mouse_matrix(data):
    global mouse_matrix, mouse_ref_point_c, mouse_ref_point_a
    basis = numpy.array([[data.matrix_tf[0], data.matrix_tf[2]], [data.matrix_tf[1], data.matrix_tf[3]]])
    mouse_matrix = basis
    mouse_ref_point_c = (data.matrix_tf[4], data.matrix_tf[5])
    mouse_ref_point_a = (data.matrix_tf[6], data.matrix_tf[7])


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

    laser_tf_listener = rospy.ServiceProxy('laser_tf', Laser_tf)


    while not rospy.is_shutdown():
        # print(hog_matrix)
        # print(hog_pc)

        hog_new_pc = PointCloud()
        hog_new_pc.header = copy.deepcopy(hog_pc_header)
        hog_new_pc.channels.append(ChannelFloat32())
        hog_new_pc.channels[0].name = "intensity"
        # print(hog_new_pc)

        if hog_matrix.any():
            # print(hog_matrix)
            for pt in hog_pc:
                point_in_matrix_form = numpy.array([[pt.x],[pt.y]])
                # print(point_in_matrix_form)
                # print(hog_matrix)

                hog_new_point = numpy.matmul(numpy.array([[1,0],[0,1]]), point_in_matrix_form)
                hog_new_pc.points.append(Point(hog_new_point[0][0], hog_new_point[1][0], 0.0))
                hog_new_pc.channels[0].values.append(.99)

            hog_pub.publish(hog_new_pc)


            mouse_new_pc = PointCloud()
            mouse_new_pc.header = copy.deepcopy(mouse_pc_header)
            mouse_new_pc.channels.append(ChannelFloat32())
            mouse_new_pc.channels[0].name = "intensity"

            # print(hog_matrix)
            # print(mouse_matrix)
            # change_basis = numpy.array([[hog_matrix[0][0], hog_matrix[0][1], mouse_matrix[0][0], mouse_matrix[0][1]],
            #                                  [hog_matrix[1][0], hog_matrix[1][1], mouse_matrix[1][0], mouse_matrix[1][1]]])
            # change_basis_rref = sympy.Matrix(change_basis).rref()
            # # print(change_basis_rref[0])
            # new_basis = change_basis_rref[0]
            # print(new_basis.row(0).col(2)[0])
            # change_basis_mouse = numpy.array([[float(new_basis.row(0).col(2)[0]), float(new_basis.row(0).col(3)[0])],
            #                                   [float(new_basis.row(1).col(2)[0]), float(new_basis.row(1).col(3)[0])]])

            # hog_ref_matrix = numpy.array([[hog_ref_point[0]], [hog_ref_point[1]]])
            # mouse_ref_matrix = numpy.array([[mouse_ref_point[0]], [mouse_ref_point[1]]])
            #
            # print(hog_ref_matrix)
            # print(inv(mouse_ref_matrix))
            #
            # change_basis_mouse = numpy.matmul(hog_ref_matrix, inv(mouse_ref_matrix))

            # print(change_basis_mouse)

            # hog_to_mouse = (float(mouse_ref_point_c[0] - hog_ref_point_c[0]), float(mouse_ref_point_c[1] - hog_ref_point_c[1]))
            # mouse_to_hog = (float(mouse_ref_point_c[0] - hog_ref_point_c[0]), float(mouse_ref_point_c[1] - hog_ref_point_c[1]))

            mouse_to_hog = (float(hog_ref_point_c[0] - mouse_ref_point_c[0]), float(hog_ref_point_c[1] - mouse_ref_point_c[1]))

            # adding all the points to the hog reference frame

            # change between reference point (right angle) and short leg in the frame it is being published
            # delta hog = delta x , delta y
            # delta_hog = ((hog_ref_point_a[0] - hog_ref_point_c[0]), (hog_ref_point_a[1] - hog_ref_point_c[1]))
            delta_mouse = ((mouse_ref_point_a[0] - mouse_ref_point_c[0]), (mouse_ref_point_a[1] - mouse_ref_point_c[1]))

            # hog_to_mouse_a = ((mouse_ref_point_a[0] + delta_hog[0]), (mouse_ref_point_a[1] + delta_hog[1]))

            # finding the angle between hog's short leg and mouse's short leg to find relative orientation
            # delta_q = ((mouse_ref_point_a[0] - mouse_ref_point_c[0]), (mouse_ref_point_a[1] - mouse_ref_point_c[1]))
            delta_q = ((hog_ref_point_a[0] - hog_ref_point_c[0]), (hog_ref_point_a[1] - hog_ref_point_c[1]))

            theta_q = math.atan2(delta_q[1], delta_q[0])
            # theta_hog = math.atan2(delta_hog[1], delta_hog[0])
            theta_mouse = math.atan2(delta_mouse[1], delta_mouse[0])

            # theta = theta_q - theta_hog
            theta = theta_q - theta_mouse
            #(theta_source, Point(delta_source[0], delta_source[1], 0), theta, theta_dest, Point(delta_dest[0], delta_dest[1], 0))
            print("theta_mouse, delta_mouse, theta, theta_q, delta_q")
            print(theta_mouse, delta_mouse, theta, theta_q, delta_q)
            print("****************************")
            print(laser_tf_listener(Point(mouse_ref_point_c[0], mouse_ref_point_c[1], 0),
                                    Point(hog_ref_point_c[0], hog_ref_point_c[1], 0),
                                    Point(mouse_ref_point_a[0], mouse_ref_point_a[1], 0),
                                    Point(hog_ref_point_a[0], hog_ref_point_a[1], 0)))
            # print(laser_tf_listener(Point(mouse_ref_point_c + (0,)), Point(hog_ref_point_c + (0,)), Point(mouse_ref_point_a + (0,)), Point(hog_ref_point_a + (0,))))
            print()
            print()


            # print(hog_to_mouse)
            for pt in mouse_pc:
                point_in_matrix_form = numpy.array([[pt.x - 4.267],[pt.y]])
                # print(point_in_matrix_form)
                # print(hog_matrix)

                # mouse_new_point = numpy.matmul(change_basis_mouse, point_in_matrix_form)
                # mouse_new_pc.points.append(Point(mouse_new_point[0][0], mouse_new_point[1][0], 0.0))
                # mouse_new_pc.points.append(Point((pt.x - mouse_to_hog[0], pt.y - mouse_to_hog[1], 0))
                translated_point = (pt.x - mouse_ref_point_c[0], pt.y - mouse_ref_point_c[1])
                rotated_point = (math.cos(theta) * translated_point[0] - math.sin(theta) * translated_point[1],
                                 math.sin(theta) * translated_point[0] + math.cos(theta) * translated_point[1])
                post_translation = (rotated_point[0] + mouse_ref_point_c[0], rotated_point[1] + mouse_ref_point_c[1])
                rotated_point = post_translation
                mouse_new_pc.points.append(Point(rotated_point[0] + mouse_to_hog[0],
                                                 rotated_point[1] + mouse_to_hog[1], 0))
                mouse_new_pc.channels[0].values.append(.99)

            mouse_pub.publish(mouse_new_pc)
            #



if __name__ == '__main__':
    listener()
