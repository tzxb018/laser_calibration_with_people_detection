#!/usr/bin/env python
import rospy
import tf
from lc.srv import Laser_tf, Detection_target
from sensor_msgs.msg import LaserScan, PointCloud2, PointCloud, ChannelFloat32
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
import copy

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
laserSettings_hog = {}
laserSettings_mouse = {}

lastLaser_hog, laser_pub_hog, laser_sub_hog = LaserScan(), [], []
lastLaser_mouse, laser_pub_mouse, laser_sub_mouse = LaserScan(), [], []


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

def callback_hog_laser(data):
    global lastLaser_hog, laser_pub_hog

    lastLaser_hog = copy.deepcopy(data)

    # print("callback laser hog")

    if "angle_increment" not in laserSettings_hog:
        laserSettings_hog["angle_min"] = data.angle_min
        laserSettings_hog["angle_max"] = data.angle_max
        laserSettings_hog["array_size"] = len(data.ranges)
        laserSettings_hog["angle_increment"] = data.angle_increment
        laserSettings_hog["range_min"] = data.range_min
        laserSettings_hog["range_max"] = data.range_max

    lastLaser_hog.header.stamp = rospy.Time.now()
    laser_pub_hog.publish(lastLaser_hog)

def callback_mouse_laser(data):
    global lastLaser_mouse, laser_pub_mouse

    lastLaser_mouse = copy.deepcopy(data)
    if "angle_increment" not in laserSettings_mouse:
        laserSettings_mouse["angle_min"] = data.angle_min
        laserSettings_mouse["angle_max"] = data.angle_max
        laserSettings_mouse["array_size"] = len(data.ranges)
        laserSettings_mouse["angle_increment"] = data.angle_increment
        laserSettings_mouse["range_min"] = data.range_min
        laserSettings_mouse["range_max"] = data.range_max

    lastLaser_mouse.header.stamp = rospy.Time.now()
    laser_pub_mouse.publish(lastLaser_mouse)

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

def publish_tf():
    global lastLaser_hog, lastLaser_mouse, laser_pub_hog
    global laser_sub_hog, laser_sub_mouse, laser_pub_mouse

    rospy.init_node('laser_tf_broadcaster')
    rate = rospy.Rate(10)

    laser_sub_hog = rospy.Subscriber("/hog/scan0", LaserScan, callback_hog_laser)
    laser_sub_mouse = rospy.Subscriber("/mouse/scan0", LaserScan, callback_mouse_laser)

    laser_target_finder = rospy.ServiceProxy('detection_target', Detection_target)

    laser_pub_hog = rospy.Publisher("/hog/updatedScan", LaserScan, queue_size = 10)
    laser_pub_mouse = rospy.Publisher("/mouse/updatedScan", LaserScan, queue_size = 10)

    # hog_sub = rospy.Subscriber("/matrix_for_hog", matrix_tf, callback_hog_matrx)
    # hog_bg_sub = rospy.Subscriber('/hog/bg_cloud', PointCloud, callback_hog_pc)
    #
    # mouse_sub = rospy.Subscriber("/matrix_for_mouse", matrix_tf, callback_mouse_matrix)
    # mouse_bg_sub = rospy.Subscriber('/mouse/bg_cloud', PointCloud, callback_mouse_pc)

    # callback_hog_matrx(rospy.wait_for_message("/matrix_for_hog", matrix_tf))
    # callback_hog_pc(rospy.wait_for_message('/hog/bg_cloud', PointCloud))
    # callback_mouse_matrix(rospy.wait_for_message("/matrix_for_mouse", matrix_tf))
    # callback_mouse_pc(rospy.wait_for_message('/mouse/bg_cloud', PointCloud))

    callback_hog_laser(rospy.wait_for_message('/hog/scan0', LaserScan))
    callback_mouse_laser(rospy.wait_for_message('mouse/scan0', LaserScan))

    # calls the service for finding the calibration target
    # print(lastLaser_hog)
    reu = laser_target_finder(lastLaser_hog)
    hog_ref_point_a = reu.point_a
    hog_ref_point_c = reu.point_c

    rev = laser_target_finder(lastLaser_mouse)
    mouse_ref_point_a = rev.point_a
    mouse_ref_point_c = rev.point_c

    print("hog a\n" + str(hog_ref_point_a))
    print("hog c\n" + str(hog_ref_point_c))
    print("mouse a\n" + str(mouse_ref_point_a))
    print("mouse c\n" + str(mouse_ref_point_c))

    # print(hog_ref_point_a)
    # print(hog_ref_point_c)
    # print(mouse_ref_point_a)
    # print(mouse_ref_point_c)

    laser_tf_listener = rospy.ServiceProxy('laser_tf', Laser_tf)

    # calls the service for finding the tfs between two frames
    ret = laser_tf_listener(Point(mouse_ref_point_c.x, mouse_ref_point_c.y, 0),
                            Point(hog_ref_point_c.x, hog_ref_point_c.y, 0),
                            Point(mouse_ref_point_a.x, mouse_ref_point_a.y, 0),
                            Point(hog_ref_point_a.x, hog_ref_point_a.y, 0))
    print("tf\n" + str(ret))
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

        # call the laser update to publish the updated laser scan
        callback_hog_laser(rospy.wait_for_message('/hog/scan0', LaserScan))
        callback_mouse_laser(rospy.wait_for_message('mouse/scan0', LaserScan))

        rate.sleep()

if __name__ == '__main__':
    publish_tf()









