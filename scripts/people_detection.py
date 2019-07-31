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


# marks = MarkerArray()
# laserinput = []
lastLaser = []
# show_time = 1
# ind_list = []
# r = float(sys.argv[4])
# center = (0, 0)
# center_index = 0
# center_angle = 0
# rate = .2
# error = 0.001
# circle_threshold = 8
# center_points = []  # stores the centers of each detected circle
# possible_triangle = [] # stores the point that make up the calibration target
laserSettings = {}
# max_range = 25.0
pc_li_hog = PointCloud()
pc_li_snake = PointCloud()
pc_li_mouse = PointCloud()
pc_display = PointCloud()
laser_pub = []
# isFinshed = True
laser_in = LaserScan()
# send_matrix = []
# laser_topic_name = ""
# max_count = 10
# min_dist = .25
# timeout_time = rospy.Duration(10.0)

def callback_hog_laser(data):
    global pc_li_hog

    pc_li_hog = make_PC_from_Laser_display(data)

def callback_mouse_laser(data):
    global pc_li_mouse

    pc_li_mouse = make_PC_from_Laser_display(data)

def callback_snake_laser(data):
    global pc_li_snake

    pc_li_snake = make_PC_from_Laser_display(data)

# def make_PC_from_Laser(laser_in):
#     cloud = []
#
#     for x in range(len(laser_in.ranges)):
#         if laser_in.ranges[x] >= laserSettings["range_max"] or laser_in.ranges[x] <= laserSettings["range_min"]:
#             cloud.append((laserSettings["range_max"] * math.cos(
#                 laserSettings["angle_min"] + x * laserSettings["angle_increment"]),
#                           laserSettings["range_max"] * math.sin(
#                               laserSettings["angle_min"] + x * laserSettings["angle_increment"]), 0))
#         else:
#             cloud.append((laser_in.ranges[x] * math.cos(
#                 laserSettings["angle_min"] + x * laserSettings["angle_increment"]), laser_in.ranges[x] * math.sin(
#                 laserSettings["angle_min"] + x * laserSettings["angle_increment"]), 0))
#     return cloud

def dist(point, center):
    square_dist = (point[0] - center[0]) ** 2 + (point[1] - center[1]) ** 2
    return square_dist ** 0.5


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

    bg_pub = rospy.Publisher('/combined', PointCloud, queue_size=10)
    tf_listen = tf.TransformListener(True)
    tf_transformer = tf.TransformerROS()

    br = tf.TransformBroadcaster()

    callback_hog_laser(rospy.wait_for_message('/hog/scan0', LaserScan))
    try:
        tf_listen.waitForTransform("/laser_hog", "/laser_snake", pc_li_hog.header.stamp, rospy.Duration(3.0))
        tf_listen.waitForTransform("/laser_hog", "/laser_mouse", pc_li_hog.header.stamp, rospy.Duration(3.0))
    except:
        pass
    snake_to_hog = tf_listen.lookupTransform("/laser_hog", "/laser_snake", pc_li_hog.header.stamp)
    # print(snake_to_hog)
    snake_matrix = tf_transformer.fromTranslationRotation(snake_to_hog[0], snake_to_hog[1])
    mouse_to_hog = tf_listen.lookupTransform("/laser_hog", "/laser_mouse", pc_li_hog.header.stamp)
    mouse_matrix = tf_transformer.fromTranslationRotation(mouse_to_hog[0], mouse_to_hog[1])

    # snake_matrix[0][1] = - snake_matrix[0][1]
    # snake_matrix[1][0] = - snake_matrix[1][0]
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

        # try:
        #     tf_listen.waitForTransform("/laser_hog", "/laser_snake", pc_li_hog.header.stamp, rospy.Duration(3.0))
        #     tf_listen.waitForTransform("/laser_hog", "/laser_mouse", pc_li_hog.header.stamp, rospy.Duration(3.0))
        # except:
        #     continue

        # try:
        #     tf_listen.waitForTransform("/laser_hog", "/laser_snake", pc_li_hog.header.stamp, rospy.Duration(1.0))
        #     tf_listen.waitForTransform("/laser_hog", "/laser_mouse", pc_li_hog.header.stamp, rospy.Duration(1.0))
        # except:
        #     continue



        # t_hog = tf_listen.getLatestCommonTime("laser_hog", "laser_hog")
        # t_mouse = tf_listen.getLatestCommonTime("/laser_hog", "/laser_mouse")

        p1 = PointStamped()
        p1.header = copy.deepcopy(pc_li_hog.header)
        # p1.pose.orientation.w = 1.0

        for pt in pc_li_hog.points:
            p1.point.x = pt.x
            p1.point.y = pt.y

            # p_in_hog = tf_listen.transformPoint("laser_hog", p1)

            pc_display.points.append(Point(p1.point.x, p1.point.y, p1.point.z))

        p1.header.frame_id = "/laser_snake"
        # print("afjoeiwfje")

        # matrix_point = tf.fromTranslationRotation(snake_to_hog)
        num_point = numpy.array([0.0,0.0,0.0,1.0])

        # pc_li_snake = tf_listen.transformPointCloud("laser_hog", pc_li_snake)
        for pt in pc_li_snake.points:
            num_point[0] = pt.x
            num_point[1] = pt.y
            num_point[2] = pt.z

            # out = numpy.dot(num_point, snake_matrix)
            out1 = numpy.dot(snake_matrix, num_point)

            # print(out)
            # print("*******")
            # print(pt)
            # print(out1)
            # print(out)
            # p1.point.x = pt.x #+ snake_to_hog[0][0]
            # p1.point.y = pt.y #+ snake_to_hog[0][1]

            # p1 = tf_listen.transformPoint("laser_hog", p1)
            # x = 0
        # tf_listen.waitForTransform("laser_hog", "laser_snake", pc_li_hog.header.stamp)
        # while True:
        #     try:
        #         # t_snake = tf_listen.getLatestCommonTime("laser_hog", "laser_snake")
        #         # p1 = tf_listen.transformPoint("laser_hog", p1)
        #         pc_transform_snake = tf_listen.transformPointCloud("laser_hog", pc_li_snake)
        #         break
        #     except Exception as e:
        #         print(e)

        # pc_display.points = pc_display.points + pc_transform_snake.points
        #     pc_display.points.append(Point(out[0] + snake_to_hog[0][0] + .18, out[1] + snake_to_hog[0][1] + .58, out[2]))
            pc_display.points.append(Point(out1[0], out1[1], out1[2]))

        for pt in pc_li_mouse.points:
            num_point[0] = pt.x
            num_point[1] = pt.y
            num_point[2] = pt.z

            # out = numpy.dot(num_point, snake_matrix)
            out2 = numpy.dot(mouse_matrix, num_point)

            # print(out)
            # print("*******")
            # print(pt)
            # print(out2)
            # print(out)
            # p1.point.x = pt.x #+ snake_to_hog[0][0]
            # p1.point.y = pt.y #+ snake_to_hog[0][1]

            # p1 = tf_listen.transformPoint("laser_hog", p1)
            # x = 0
            # tf_listen.waitForTransform("laser_hog", "laser_snake", pc_li_hog.header.stamp)
            # while True:
            #     try:
            #         # t_snake = tf_listen.getLatestCommonTime("laser_hog", "laser_snake")
            #         # p1 = tf_listen.transformPoint("laser_hog", p1)
            #         pc_transform_snake = tf_listen.transformPointCloud("laser_hog", pc_li_snake)
            #         break
            #     except Exception as e:
            #         print(e)

            # pc_display.points = pc_display.points + pc_transform_snake.points
            #     pc_display.points.append(Point(out[0] + snake_to_hog[0][0] + .18, out[1] + snake_to_hog[0][1] + .58, out[2]))
            pc_display.points.append(Point(out2[0], out2[1], out2[2]))





        # p1.header.frame_id = "/laser_mouse"

        # for pt in pc_li_snake.points:
        #     p1.point.x = pt.x + mouse_to_hog[0][0]
        #     p1.point.y = pt.y + mouse_to_hog[0][1]
        #     pc_display.points.append(p1.point)

            # pc_display.points.append(pt)
        #
        # for pt in pc_li_mouse.points:
        #     pc_display.points.append(pt)

        pc_display.header = copy.deepcopy(pc_li_hog.header)
        pc_display.header.frame_id = "map"

        bg_pub.publish(pc_display)
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
