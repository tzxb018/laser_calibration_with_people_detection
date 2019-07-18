#!/usr/bin/env python
import rospy
import tf
from lc.srv import Laser_tf
from sensor_msgs.msg import LaserScan, PointCloud2, PointCloud, ChannelFloat32
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3

import numpy
from lc.msg import matrix_tf



laser_hog, laser_mouse = [], []

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

def callback_mouse_pc(data):
    global mouse_pc, mouse_pc_header

    mouse_pc_header = data.header
    mouse_pc = data.points
#
# def updateLaserHog(data):
#     global laser_hog
#
#     laser_hog = data
#
# def updateLaserMouse(data):
#     global laser_mouse
#
#     laser_mouse = data


if __name__ == '__main__':
    rospy.init_node('laser_tf_broadcaster')
    r = rospy.Rate(10)
    #
    # laserList_mouse = rospy.Subscriber("/mouse/scan0", LaserScan, updateLaserMouse)
    # laserList_hog = rospy.Subscriber("/hog/scan0", LaserScan, updateLaserHog)

    laser_tf_listener = rospy.ServiceProxy('laser_tf', Laser_tf)

    # hog_sub = rospy.Subscriber("/matrix_for_hog", matrix_tf, callback_hog_matrx)
    # hog_bg_sub = rospy.Subscriber('/hog/bg_cloud', PointCloud, callback_hog_pc)
    #
    # mouse_sub = rospy.Subscriber("/matrix_for_mouse", matrix_tf, callback_mouse_matrix)
    # mouse_bg_sub = rospy.Subscriber('/mouse/bg_cloud', PointCloud, callback_mouse_pc)

    callback_hog_matrx(rospy.wait_for_message("/matrix_for_hog", matrix_tf))
    callback_hog_pc(rospy.wait_for_message('/hog/bg_cloud', PointCloud))
    callback_mouse_matrix(rospy.wait_for_message("/matrix_for_mouse", matrix_tf))
    callback_mouse_pc(rospy.wait_for_message('/mouse/bg_cloud', PointCloud))

    ret = laser_tf_listener(Point(mouse_ref_point_c[0], mouse_ref_point_c[1], 0),
                      Point(hog_ref_point_c[0], hog_ref_point_c[1], 0),
                      Point(mouse_ref_point_a[0], mouse_ref_point_a[1], 0),
                      Point(hog_ref_point_a[0], hog_ref_point_a[1], 0))

    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        # first tf
        br.sendTransform((ret.source_translation.x, ret.source_translation.y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, ret.source_theta),
                         rospy.Time.now(),
                         "laser_hog_target",
                         "laser_hog")

        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, ret.theta),
                         rospy.Time.now(),
                         "laser_mouse_target",
                         "laser_hog_target")

        br.sendTransform((ret.dest_translation.x, ret.dest_translation.y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, ret.dest_theta),
                         rospy.Time.now(),
                         "laser_mouse",
                         "laser_mouse_target")
        r.sleep()








