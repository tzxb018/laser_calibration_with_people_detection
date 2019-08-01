#!/usr/bin/env python
import rospy
import sys
import tf
import copy
from lc.srv import Detection_target
import math
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan, PointCloud2, PointCloud, ChannelFloat32
import sensor_msgs.point_cloud2 as pc
import laser_geometry as lp
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseStamped, PointStamped
import numpy

lastLaser = []
laserSettings = {}
pc_li_hog = PointCloud()
pc_li_snake = PointCloud()
pc_li_mouse = PointCloud()
pc_display = PointCloud()
laser_pub = []
laser_in = LaserScan()
old_data_hog1, old_data_hog2, old_data_hog3, old_data_hog4 = [], [], [], []
old_data_snake1, old_data_snake2, old_data_snake3, old_data_snake4 = [], [], [], []
old_data_mouse1, old_data_mouse2, old_data_mouse3, old_data_mouse4 = [], [], [], []
change_in_time = .05
max_range = 25.0
pc_li_hog_all = PointCloud()
pc_li_snake_all = PointCloud()
pc_li_mouse_all = PointCloud()
pc_display_all = PointCloud()


def callback_hog_laser_init(data):
    global pc_li_hog
    pc_li_hog = make_PC_from_Laser_display(data)

def callback_hog_laser(data):
    global pc_li_hog, old_data_hog1, old_data_hog2, old_data_hog3, old_data_hog4, pc_li_hog_all

    # have 5 arrays store 5 most recent laser scans
    # find variance of each index in 5 frames
    # if variance is greater than threshold, then detect as movmenet, else nah
    points_to_be_converted = []
    print("hog")

    # if there is history of the laser scans (should work besides the very first laser scan)
    if old_data_hog1 and old_data_hog2 and old_data_hog3 and old_data_hog4:

        # convert to lists
        data.ranges = list(data.ranges)
        old_data_hog1.ranges = list(old_data_hog1.ranges)
        old_data_hog2.ranges = list(old_data_hog2.ranges)
        old_data_hog3.ranges = list(old_data_hog3.ranges)
        old_data_hog4.ranges = list(old_data_hog4.ranges)

        # iterating through each angle of the laser
        for i in range(0, len(data.ranges)):
            # if abs(data.ranges[i] - old_data_hog1.ranges[i]) >= change_in_time:
            #     points_to_be_converted.append(data.ranges[i])
            # else:
            #     points_to_be_converted.append(numpy.inf)

            # calculate the variance of the points of the 5 frames at each angle
            variance_between_points = numpy.var([old_data_hog1.ranges[i], old_data_hog2.ranges[i], old_data_hog3.ranges[i],
                          old_data_hog4.ranges[i], data.ranges[i]])
            print(variance_between_points)

            # if the variance exists and is greater than a threshold, add the point to another array
            # this determines if the point is part of a moving object or not
            if not math.isnan(variance_between_points) and variance_between_points >= change_in_time:
                points_to_be_converted.append(data.ranges[i])
            else:

                # append inf to make sure the point cloud does not display this angle
                points_to_be_converted.append(numpy.inf)
    else:
        print("all new")

        # add old data to the arrays as a placeholder
        old_data_hog1 = data
        old_data_hog2 = data
        old_data_hog3 = data
        old_data_hog4 = data
        points_to_be_converted = data.ranges

    # slide the histories of the laser scan one more array down (keeps the 5 most recent laser scans)
    old_data_hog4 = old_data_hog3
    old_data_hog3 = old_data_hog2
    old_data_hog2 = old_data_hog1
    old_data_hog1 = data

    # init another laser scan just for the points that are tracked as movement
    change_hog = LaserScan()
    change_hog = copy.deepcopy(data)
    change_hog.ranges = points_to_be_converted
    # print(len(points_to_be_converted))

    # make point clouds for the movement
    pc_li_hog = make_PC_from_Laser_display(change_hog)

    # make point clouds for the whole laser scan
    pc_li_hog_all = make_PC_from_Laser_display(data)

def callback_mouse_laser(data):
    global pc_li_mouse, old_data_mouse1, old_data_mouse2, old_data_mouse3, old_data_mouse4, pc_li_mouse_all

    # have 5 arrays store 5 most recent laser scans
    # find variance of each index in 5 frames
    # if variance is greater than threshold, then detect as movmenet, else nah
    points_to_be_converted = []
    print("mouse")

    # if there is history of the laser scans (should work besides the very first laser scan)
    if old_data_mouse1 and old_data_mouse2 and old_data_mouse3 and old_data_mouse4:

        # convert to lists
        data.ranges = list(data.ranges)
        old_data_mouse1.ranges = list(old_data_mouse1.ranges)
        old_data_mouse2.ranges = list(old_data_mouse2.ranges)
        old_data_mouse3.ranges = list(old_data_mouse3.ranges)
        old_data_mouse4.ranges = list(old_data_mouse4.ranges)

        # iterating through each angle of the laser
        for i in range(0, len(data.ranges)):
            # if abs(data.ranges[i] - old_data_hog1.ranges[i]) >= change_in_time:
            #     points_to_be_converted.append(data.ranges[i])
            # else:
            #     points_to_be_converted.append(numpy.inf)

            # calculate the variance of the points of the 5 frames at each angle
            variance_between_points = numpy.var(
                [old_data_mouse1.ranges[i], old_data_mouse2.ranges[i], old_data_mouse3.ranges[i],
                 old_data_mouse4.ranges[i], data.ranges[i]])
            print(variance_between_points)

            # if the variance exists and is greater than a threshold, add the point to another array
            # this determines if the point is part of a moving object or not
            if not math.isnan(variance_between_points) and variance_between_points >= change_in_time:
                points_to_be_converted.append(data.ranges[i])
            else:

                # append inf to make sure the point cloud does not display this angle
                points_to_be_converted.append(numpy.inf)
    else:
        print("all new")

        # add old data to the arrays as a placeholder
        old_data_mouse1 = data
        old_data_mouse2 = data
        old_data_mouse3 = data
        old_data_mouse4 = data
        points_to_be_converted = data.ranges

    # slide the histories of the laser scan one more array down (keeps the 5 most recent laser scans)
    old_data_mouse4 = old_data_mouse3
    old_data_mouse3 = old_data_mouse2
    old_data_mouse2 = old_data_mouse1
    old_data_mouse1 = data

    # init another laser scan just for the points that are tracked as movement
    change_mouse = LaserScan()
    change_mouse = copy.deepcopy(data)
    change_mouse.ranges = points_to_be_converted
    # print(len(points_to_be_converted))

    # make point clouds for the movement
    pc_li_mouse = make_PC_from_Laser_display(change_mouse)

    # make point clouds for the whole laser scan
    pc_li_mouse_all = make_PC_from_Laser_display(data)


def callback_snake_laser(data):
    global pc_li_snake, old_data_snake1, old_data_snake2, old_data_snake3, old_data_snake4, pc_li_snake_all

    # have 5 arrays store 5 most recent laser scans
    # find variance of each index in 5 frames
    # if variance is greater than threshold, then detect as movmenet, else nah
    points_to_be_converted = []
    print("snake")

    # if there is history of the laser scans (should work besides the very first laser scan)
    if old_data_snake1 and old_data_snake2 and old_data_snake3 and old_data_snake4:

        # convert to lists
        data.ranges = list(data.ranges)
        old_data_snake1.ranges = list(old_data_snake1.ranges)
        old_data_snake2.ranges = list(old_data_snake2.ranges)
        old_data_snake3.ranges = list(old_data_snake3.ranges)
        old_data_snake4.ranges = list(old_data_snake4.ranges)

        # iterating through each angle of the laser
        for i in range(0, len(data.ranges)):
            # if abs(data.ranges[i] - old_data_hog1.ranges[i]) >= change_in_time:
            #     points_to_be_converted.append(data.ranges[i])
            # else:
            #     points_to_be_converted.append(numpy.inf)

            # calculate the variance of the points of the 5 frames at each angle
            variance_between_points = numpy.var(
                [old_data_snake1.ranges[i], old_data_snake2.ranges[i], old_data_snake3.ranges[i],
                 old_data_snake4.ranges[i], data.ranges[i]])
            print(variance_between_points)

            # if the variance exists and is greater than a threshold, add the point to another array
            # this determines if the point is part of a moving object or not
            if not math.isnan(variance_between_points) and variance_between_points >= change_in_time:
                points_to_be_converted.append(data.ranges[i])
            else:

                # append inf to make sure the point cloud does not display this angle
                points_to_be_converted.append(numpy.inf)
    else:
        print("all new")

        # add old data to the arrays as a placeholder
        old_data_snake1 = data
        old_data_snake2 = data
        old_data_snake3 = data
        old_data_snake4 = data
        points_to_be_converted = data.ranges

    # slide the histories of the laser scan one more array down (keeps the 5 most recent laser scans)
    old_data_snake4 = old_data_snake3
    old_data_snake3 = old_data_snake2
    old_data_snake2 = old_data_snake1
    old_data_snake1 = data

    # init another laser scan just for the points that are tracked as movement
    change_snake = LaserScan()
    change_snake = copy.deepcopy(data)
    change_snake.ranges = points_to_be_converted
    # print(len(points_to_be_converted))

    # make point clouds for the movement
    pc_li_snake = make_PC_from_Laser_display(change_snake)

    # make point clouds for the whole laser scan
    pc_li_snake_all = make_PC_from_Laser_display(data)


def make_PC_from_Laser_display(laser_in):

    # Initialize a point cloud object
    pc_out = PointCloud()
    # Converts the message from a LaserScan to a Point Cloud
    projection = lp.LaserProjection()

    cloud = projection.projectLaser(laser_in)  # ,channel_options = 0x04)
    # Convert it to individual points
    cloud = pc.read_points(cloud, field_names=("x", "y", "z"))  # ,"distances"))
    cloud = list(cloud)

    pc_out.header = copy.deepcopy(laser_in.header)
    pc_out.channels.append(ChannelFloat32())
    pc_out.channels[0].name = "intensity"
    # Format each Point Cloud into a x,y,z coordinates
    for cc in cloud:
        pc_out.points.append(Point(cc[0], cc[1], cc[2]))
        pc_out.channels[0].values.append(.99)
    return pc_out


def combine_lasers():
    global pc_li_hog, pc_display, pc_li_mouse, pc_li_snake

    bg_pub = rospy.Publisher('/combined_movement', PointCloud, queue_size=10)
    bg_pub_all = rospy.Publisher('/combined', PointCloud, queue_size=10)
    tf_listen = tf.TransformListener(True)
    tf_transformer = tf.TransformerROS()

    br = tf.TransformBroadcaster()


    callback_hog_laser_init(rospy.wait_for_message('/hog/scan0', LaserScan))
    try:
        tf_listen.waitForTransform("/laser_hog", "/laser_snake", pc_li_hog.header.stamp, rospy.Duration(2.0))
        tf_listen.waitForTransform("/laser_hog", "/laser_mouse", pc_li_hog.header.stamp, rospy.Duration(2.0))
    except:
        pass
    snake_to_hog = tf_listen.lookupTransform("/laser_hog", "/laser_snake", pc_li_hog.header.stamp)
    snake_matrix = tf_transformer.fromTranslationRotation(snake_to_hog[0], snake_to_hog[1])
    mouse_to_hog = tf_listen.lookupTransform("/laser_hog", "/laser_mouse", pc_li_hog.header.stamp)
    mouse_matrix = tf_transformer.fromTranslationRotation(mouse_to_hog[0], mouse_to_hog[1])

    print(snake_to_hog)
    print(snake_matrix)
    print(mouse_to_hog)
    # return

    while not rospy.is_shutdown():

        # call the laser update to publish the updated laser scan
        callback_hog_laser(rospy.wait_for_message('/hog/scan0', LaserScan))
        callback_mouse_laser(rospy.wait_for_message('/mouse/scan0', LaserScan))
        callback_snake_laser(rospy.wait_for_message('/snake/scan0', LaserScan))
        pc_display = PointCloud()
        pc_display_all = PointCloud()

        p1 = PointStamped()
        p1.header = copy.deepcopy(pc_li_hog.header)

        for pt in pc_li_hog.points:
            p1.point.x = pt.x
            p1.point.y = pt.y

            pc_display.points.append(Point(p1.point.x, p1.point.y, p1.point.z))

        for pt in pc_li_hog_all.points:
            p1.point.x = pt.x
            p1.point.y = pt.y

            pc_display_all.points.append(Point(p1.point.x, p1.point.y, p1.point.z))

        p1.header.frame_id = "/laser_snake"

        num_point = numpy.array([0.0,0.0,0.0,1.0])

        for pt in pc_li_snake.points:
            num_point[0] = pt.x
            num_point[1] = pt.y
            num_point[2] = pt.z

            out1 = numpy.dot(snake_matrix, num_point)

            pc_display.points.append(Point(out1[0], out1[1], out1[2]))

        for pt in pc_li_snake_all.points:
            num_point[0] = pt.x
            num_point[1] = pt.y
            num_point[2] = pt.z

            out1 = numpy.dot(snake_matrix, num_point)

            pc_display_all.points.append(Point(out1[0], out1[1], out1[2]))

        for pt in pc_li_mouse.points:
            num_point[0] = pt.x
            num_point[1] = pt.y
            num_point[2] = pt.z

            out2 = numpy.dot(mouse_matrix, num_point)

            pc_display.points.append(Point(out2[0], out2[1], out2[2]))

        for pt in pc_li_mouse_all.points:
            num_point[0] = pt.x
            num_point[1] = pt.y
            num_point[2] = pt.z

            out2 = numpy.dot(mouse_matrix, num_point)

            pc_display_all.points.append(Point(out2[0], out2[1], out2[2]))

        pc_display.header = copy.deepcopy(pc_li_hog.header)
        pc_display.header.frame_id = "map"

        pc_display_all.header = copy.deepcopy(pc_li_hog.header)
        pc_display_all.header.frame_id = "map"
        print(len(pc_display_all.points))
        bg_pub.publish(pc_display)
        bg_pub_all.publish(pc_display_all)
        # print(pc_display)
        print(len(pc_display.points))
        print("==================================================================")

def ankle_tracking():
    global pc_li_hog

    laserList = rospy.Subscriber("/hog/scan0", LaserScan, callback_hog_laser)

    previous_points = []
    while not rospy.is_shutdown():

        previous_points = pc_li_hog.points




if __name__ == '__main__':
    rospy.init_node('people_detection')
    combine_lasers()
