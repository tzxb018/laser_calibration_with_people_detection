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
import random
from collections import deque

pc_li_hog = PointCloud()
pc_li_snake = PointCloud()
pc_li_mouse = PointCloud()
pc_display = PointCloud()
pc_li_hog_all = PointCloud()
pc_li_snake_all = PointCloud()
pc_li_mouse_all = PointCloud()
pc_display_all = PointCloud()
laser_pub = []
laser_in = LaserScan()
old_data_hog1, old_data_hog2, old_data_hog3, old_data_hog4 = [], [], [], []
old_data_snake1, old_data_snake2, old_data_snake3, old_data_snake4 = [], [], [], []
old_data_mouse1, old_data_mouse2, old_data_mouse3, old_data_mouse4 = [], [], [], []
change_in_time = .05
max_range = 25.0
rate = .1
circle_threshold = 12
rand_color = [(random.randint(0, 254) / 255.0, random.randint(0, 254) / 255.0, random.randint(0, 254) / 255.0) for a in
              range(0, 200)]
show_time = 1
r = .07
queue_size = 15
within_margin = .08
location_history = deque()
last_time_stamp = 0

def callback_hog_laser_init(data):
    global pc_li_hog
    pc_li_hog = make_PC_from_Laser_display(data)


def callback_hog_laser(data):
    global pc_li_hog, old_data_hog1, old_data_hog2, old_data_hog3, old_data_hog4, pc_li_hog_all

    # have 5 arrays store 5 most recent laser scans
    # find variance of each index in 5 frames
    # if variance is greater than threshold, then detect as movmenet, else nah
    points_to_be_converted = []
    # print("hog")

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
            # print(variance_between_points)

            # if the variance exists and is greater than a threshold, add the point to another array
            # this determines if the point is part of a moving object or not
            if not math.isnan(variance_between_points) and variance_between_points >= change_in_time:
                points_to_be_converted.append(data.ranges[i])
            else:

                # append inf to make sure the point cloud does not display this angle
                points_to_be_converted.append(numpy.inf)
    else:
        # print("all new")

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
    # print("mouse")

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
            # print(variance_between_points)

            # if the variance exists and is greater than a threshold, add the point to another array
            # this determines if the point is part of a moving object or not
            if not math.isnan(variance_between_points) and variance_between_points >= change_in_time:
                points_to_be_converted.append(data.ranges[i])
            else:

                # append inf to make sure the point cloud does not display this angle
                points_to_be_converted.append(numpy.inf)
    else:
        # print("all new")

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
    # print("snake")

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
            # print(variance_between_points)

            # if the variance exists and is greater than a threshold, add the point to another array
            # this determines if the point is part of a moving object or not
            if not math.isnan(variance_between_points) and variance_between_points >= change_in_time:
                points_to_be_converted.append(data.ranges[i])
            else:

                # append inf to make sure the point cloud does not display this angle
                points_to_be_converted.append(numpy.inf)
    else:
        # print("all new")

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
        showCircles()
        print("==================================================================")


# takes in the laser scan and finds a cluster of laser points that could be a circle
# returns a list of indexes (each representing the angle from the laser) and the possible center of the circle
# important to note that this center will change with gradient descent in another function (not the true center)
def getCluster(pc_li, beginInd, endInd):

    stopInd = endInd
    startInd = beginInd
    if stopInd < startInd:
        tmp = stopInd
        stopInd = startInd
        startInd = tmp

    # make sure range is in bounds
    if startInd < 0:
        startInd = 0
    if stopInd > len(pc_li.points) - 1:
        stopInd = len(pc_li.points) - 1

    # list of indexes that are in the cluster
    list_inds = []

    # init the midpoint (will be the first point)
    test_midpoint = pc_li.points[startInd]
    # FOR TESTING
    midPointArr = []
    midpoint = (0, 0, 0)

    # go through the points on the laser scan to see how big the group will get
    for cind in range(startInd + 1, stopInd):

        ckind = cind - startInd

        # the point being checked
        testPoint = (pc_li.points[cind])

        # finding the midpoint between the tested point and the mid point
        test_midpoint.x = (ckind * test_midpoint.x + testPoint.x) / (ckind + 1)
        test_midpoint.y = (ckind * test_midpoint.y + testPoint.y) / (ckind + 1)
        test_midpoint.z = (ckind * test_midpoint.z + testPoint.z) / (ckind + 1)
        # test_midpoint = (
        #     (ckind * test_midpoint.x + testPoint.x) / (ckind + 1),
        #     (ckind * test_midpoint.y + testPoint.y) / (ckind + 1),
        #     (ckind * test_midpoint.z + testPoint.z) / (ckind + 1))

        # extend the group to the next point if the tested point is within the length of the radius to the midpoint
        # previous point is within a close proximity to the next point

        proximity = .039  # arc = 2 * max_range (10) * sin ( angle/2)

        #
        if dist_from_center(pc_li.points[cind - 1], pc_li.points[cind]) <= proximity:
            midpoint = test_midpoint
            midPointArr.append(test_midpoint)
            list_inds.append(cind)

        # if the distance between the tested point and the middle is bigger than the radius (plus error) then the group ends
        else:
            break

    # return the list of indexes in the group and the purposed midpoint of the possible circle (will be changed later with gradient descent)
    return list_inds, midpoint, midPointArr


def dist_from_center(point, center):
    point = tuple_to_point(point)
    center = tuple_to_point(center)
    square_dist = (point.x - center.x) ** 2 + (point.y - center.y) ** 2
    return square_dist ** 0.5


def tuple_to_point(pt):
    try:
        return Point(pt[0], pt[1], 0)
    except:
        return pt


# update_center will find the true center of the possible circle using gradient descent
def update_center(points):
    global center, r, isFinshed

    grad_center_arr = []

    if len(points) > 0:
        new_center = center
        grad_center_arr.append(new_center)
        gradient = tuple_to_point((1, 1))

        # print(points)
        while dist_from_center(tuple_to_point((0, 0)), gradient) > 0.001:
            gradient = tuple_to_point((0, 0))
            for pt in points:
                r_prime = dist_from_center(pt, new_center)
                dldk = (r - r_prime) * ((pt.y - new_center.y) / r_prime)
                dldh = (r - r_prime) * ((pt.x - new_center.x) / r_prime)
                gradient = tuple_to_point((gradient.x + dldh, gradient.y + dldk))

            gradient = tuple_to_point((gradient.x / len(points), gradient.y / len(points)))
            new_center = tuple_to_point((new_center.x - rate * gradient.x, new_center.y - rate * gradient.y))
            grad_center_arr.append(new_center)

        isFinshed = True
        return new_center, grad_center_arr
    else:
        isFinshed = True
        return


def showCircles():
    global ankleMarks
    global ind_list, center, center_index
    global center_points, wthin_margin
    global pc_display
    global location_history, last_time_stamp

    # initialize the subscribers and the publishers for the laser, point cloud, tf, and the markers
    ankleMarkers = rospy.Publisher("/ankles", MarkerArray, queue_size=10)

    # the MarkerArray will store all the circles to be put on Rviz
    testMarks = MarkerArray()

    center_points = []

    # possible laser point clusters that could make up a circle
    clusters = []

    # init the angle index being checked (start at 0)
    center_index = 0

    # Updating the centers for each group to find the correct center of the circle using gradient descent
    id = 0  # separate id used for each marker in the marker array
    midPointArr1 = []
    grad_center_arr1 = []

    group_index = 0

    # loop through all the possible indexes
    while center_index < len(pc_display.points):

        # list of indexes making up the cluster
        ind_list = []

        # finding where to start the cluster (if the point cloud is not inf)
        # if not math.isinf(lastLaser.ranges[center_index]):
        #  obtaining the list of indexes making up the cluster and the possible center of the circle
        ind_list, midrange, midPointArr1 = getCluster(pc_display, center_index, len(
            pc_display.points))  # search to the whole scan until the shape is gone

        # points needed to be considered a circle threshold
        if len(ind_list) > circle_threshold:

            # FOR TESTING: Marking each cluster with a different color
            # rand_color = (random.randint(0,254)/255.0, random.randint(0,254)/255.0, random.randint(0,254)/255.0)
            # random.uniform(0,1), random.uniform(0,1), random.uniform(0,1), 1)
            # print(rand_color)
            # for ind in ind_list:

                # rgba = ColorRGBA(.23, .62, .67, 1)
                #
                # # add a new marker to the marker array
                # testMarks.markers.append(Marker())
                #
                # # keep the marker ids unique
                # testMarks.markers[-1].id = id
                #
                # # determining how long the markers will stay up in Rviz
                # testMarks.markers[-1].lifetime = rospy.Duration(show_time)
                #
                # # postioning will be relative to the tf of the laser
                # testMarks.markers[-1].pose = Pose(
                #     # Point(updated_center[0] + trans[0], updated_center[1] + trans[1], 0 + trans[2]),
                #     pc_display.points[ind],
                #     Quaternion(0, 0, 0, 1))
                #
                # testMarks.markers[-1].type = Marker.SPHERE
                # testMarks.markers[-1].scale = Vector3(.03, .03, .03)
                # testMarks.markers[-1].action = 0
                # testMarks.markers[-1].color = ColorRGBA(rand_color[group_index % 200][0],
                #                                         rand_color[group_index % 200][1],
                #                                         rand_color[group_index % 200][2], 1)
                # testMarks.markers[-1].header = pc_display.header  # Header(frame_id="/map")
                # testMarks.markers[-1].ns = "clusters"
                # id += 1  # keep the ids unique

            # if the shape can be considered to be a circle, then add the midpoint (key) and the index list making up the portion of the circle
            clusters.append((midrange, ind_list))

            # print("midpoints")
            # print(midPointArr1)

            # print(clusters[-1])
            # FOR TESTING
            # checking where the midpoints are for each possible circle
            # for mp in midPointArr1:

            #     # print(mp)
            #     # add a new marker to the marker array
            #     testMarks.markers.append(Marker())
            #
            #     # keep the marker ids unique
            #     testMarks.markers[-1].id = id
            #
            #     # determining how long the markers will stay up in Rviz
            #     testMarks.markers[-1].lifetime = rospy.Duration(show_time)
            #
            #     # find the tf of the laser to the parent map
            #     # this tf will be used to translate the circles to match the laser's map
            #     # try:
            #     #     (trans, rot) = tf_listen.lookupTransform('/map', '/laser_hog', rospy.Time(0))
            #     # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     #     continue
            #
            #     # postioning will be relative to the tf of the laser
            #     testMarks.markers[-1].pose = Pose(
            #         # Point(updated_center[0] + trans[0], updated_center[1] + trans[1], 0 + trans[2]),
            #         Point(mp.x, mp.y, 0.5),
            #         Quaternion(0, 0, 0, 1))
            #
            #     testMarks.markers[-1].type = Marker.SPHERE
            #     testMarks.markers[-1].scale = Vector3(.005, .005, .01)
            #     testMarks.markers[-1].action = 0
            #     testMarks.markers[-1].color = ColorRGBA(.33, 0, 1, 1)
            #     testMarks.markers[-1].header = pc_display.header  # Header(frame_id="/map")
            #     testMarks.markers[-1].ns = "midpoints"
            #     id += 1  # keep the ids unique
            #
            # # FOR TESTING: marking the last midpoint
            # testMarks.markers.append(Marker())
            # testMarks.markers[-1].id = id
            #
            # testMarks.markers[-1].pose = Pose(
            #     # Point(updated_center[0] + trans[0], updated_center[1] + trans[1], 0 + trans[2]),
            #     Point(midPointArr1[-1].x, midPointArr1[-1].y, 0.5),
            #     Quaternion(0, 0, 0, 1))
            # testMarks.markers[-1].lifetime = rospy.Duration(show_time)
            #
            # testMarks.markers[-1].type = Marker.SPHERE
            # testMarks.markers[-1].scale = Vector3(.008, .008, .01)
            # testMarks.markers[-1].action = 0
            # testMarks.markers[-1].color = ColorRGBA(.4, 0, .9, 1)
            # testMarks.markers[-1].header = pc_display.header  # Header(frame_id="/map")
            # testMarks.markers[-1].ns = "last_midpoint"
            # id += 1  # keep the ids unique

            point_for_update = []

            for i in ind_list:
                # add the x,y,z rectangular point into the array for use in update_center
                point_for_update.append((pc_display.points[i]))

            # print(point_for_update)

            center = midPointArr1[-1]

            # updated center with the given points in this cluster
            updated_center, grad_center_arr1 = update_center(point_for_update)

            within_circle = True
            for i in ind_list:
                if not (r - within_margin <= dist_from_center(pc_display.points[i], updated_center) <= r + within_margin):
                    within_circle = False
            # # FOR TESTING: drawing the margin circles
            #
            # # add a new marker to the marker array
            # testMarks.markers.append(Marker())
            #
            # # keep the marker ids unique
            # testMarks.markers[-1].id = id
            #
            # # determining how long the markers will stay up in Rviz
            # testMarks.markers[-1].lifetime = rospy.Duration(show_time)
            #
            # # find the tf of the laser to the parent map
            # # this tf will be used to translate the circles to match the laser's map
            # # try:
            # #     (trans, rot) = tf_listen.lookupTransform('/laser_hog', '/map', rospy.Time(0))
            # # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # #     continue
            #
            # # postioning will be relative to the tf of the laser
            # testMarks.markers[-1].pose = Pose(
            #     # Point(updated_center[0] + trans[0], updated_center[1] + trans[1], 0 + trans[2]),
            #     Point(updated_center.x, updated_center.y, 0),
            #     Quaternion(0, 0, 0, 1))
            #
            # testMarks.markers[-1].type = Marker.SPHERE
            # testMarks.markers[-1].scale = Vector3(2 * r + within_margin, 2 * r + within_margin, .5)
            # testMarks.markers[-1].action = 0
            # testMarks.markers[-1].color = ColorRGBA(1, 1, 1, 1)
            # testMarks.markers[-1].header = pc_display.header  # Header(frame_id="/map")
            # testMarks.markers[-1].ns = "margin"
            # id += 1  # keep the ids unique
            #
            # # add a new marker to the marker array
            # testMarks.markers.append(Marker())
            #
            # # keep the marker ids unique
            # testMarks.markers[-1].id = id
            #
            # # determining how long the markers will stay up in Rviz
            # testMarks.markers[-1].lifetime = rospy.Duration(show_time)
            #
            # # postioning will be relative to the tf of the laser
            # testMarks.markers[-1].pose = Pose(
            #     Point(updated_center.x, updated_center.y, 0),
            #     Quaternion(0, 0, 0, 1))
            #
            # testMarks.markers[-1].type = Marker.SPHERE
            # testMarks.markers[-1].scale = Vector3(2 * r - within_margin, 2 * r - within_margin, .01)
            # testMarks.markers[-1].action = 0
            # testMarks.markers[-1].color = ColorRGBA(.0, .9, .0, .1)
            # testMarks.markers[-1].header = pc_display.header  # Header(frame_id="/map")
            # testMarks.markers[-1].ns = "margin"
            # id += 1  # keep the ids unique
            if within_circle:

                # set a minimum range away from the laser to detect circles
                # this will remove alot of the false positives we are getting that are close to the laser
                # (assume that targets are not within 1 meter)
                if dist_from_center((0, 0), updated_center) >= .1:

                    # queuing the position of the circle
                    if len(location_history) < queue_size:
                        location_history.append(updated_center)
                    else:
                        location_history.popleft()
                        location_history.append(updated_center)

                    # FOR TESTING
                    # Display the grad descent process
                    # for pt in grad_center_arr1:
                    #     # add a new marker to the marker array
                    #     testMarks.markers.append(Marker())
                    #
                    #     # keep the marker ids unique
                    #     testMarks.markers[-1].id = id
                    #
                    #     # determining how long the markers will stay up in Rviz
                    #     testMarks.markers[-1].lifetime = rospy.Duration(show_time)
                    #
                    #     # postioning will be relative to the tf of the laser
                    #     testMarks.markers[-1].pose = Pose(
                    #         # Point(updated_center[0] + trans[0], updated_center[1] + trans[1], 0 + trans[2]),
                    #         Point(pt.x, pt.y, 0.5),
                    #         Quaternion(0, 0, 0, 1))
                    #     testMarks.markers[-1].type = Marker.SPHERE
                    #     testMarks.markers[-1].scale = Vector3(.005, .005, .01)
                    #     testMarks.markers[-1].action = 0
                    #     testMarks.markers[-1].color = ColorRGBA(.3, .5, 1, 1)
                    #     testMarks.markers[-1].header = pc_display.header  # Header(frame_id="/map")
                    #     testMarks.markers[-1].ns = "grad_descent"
                    #     id += 1  # keep the ids unique

                    # add a new marker to the marker array
                    # testMarks.markers.append(Marker())
                    #
                    # # keep the marker ids unique
                    # testMarks.markers[-1].id = id
                    #
                    # # determining how long the markers will stay up in Rviz
                    # testMarks.markers[-1].lifetime = rospy.Duration(show_time)
                    #
                    # # postioning will be relative to the tf of the laser
                    # testMarks.markers[-1].pose = Pose(
                    #     # Point(updated_center[0] + trans[0], updated_center[1] + trans[1], 0 + trans[2]),
                    #     Point(grad_center_arr1[-1].x, grad_center_arr1[-1].y, 0.5),
                    #     Quaternion(0, 0, 0, 1))
                    # testMarks.markers[-1].type = Marker.SPHERE
                    # testMarks.markers[-1].scale = Vector3(.008, .008, .01)
                    # testMarks.markers[-1].action = 0
                    # testMarks.markers[-1].color = ColorRGBA(.2, .5, .9, 1)
                    # testMarks.markers[-1].header = pc_display.header  # Header(frame_id="/map")
                    # testMarks.markers[-1].ns = "grad_descent"
                    # id += 1  # keep the ids unique

        group_index += 1

        # iterate to the next laser point available
        center_index += len(ind_list) + 1

    if int(pc_display.header.stamp.secs) - last_time_stamp < 0:
        location_history = deque()

    last_time_stamp = int(pc_display.header.stamp.secs)
    # printing each ankle in the queue
    for cir in location_history:
        # add a new marker to the marker array
        testMarks.markers.append(Marker())

        # keep the marker ids unique
        testMarks.markers[-1].id = id

        # determining how long the markers will stay up in Rviz
        testMarks.markers[-1].lifetime = rospy.Duration(show_time)

        # postioning will be relative to the tf of the laser
        testMarks.markers[-1].pose = Pose(
            # Point(updated_center[0] + trans[0], updated_center[1] + trans[1], 0 + trans[2]),
            Point(cir.x, cir.y, 0),
            Quaternion(0, 0, 0, 1))

        testMarks.markers[-1].type = Marker.SPHERE
        testMarks.markers[-1].scale = Vector3(2 * r, 2 * r, -.02)
        testMarks.markers[-1].action = 0
        testMarks.markers[-1].color = ColorRGBA(.3, 1, 1, 1)
        testMarks.markers[-1].header = pc_display.header  # Header(frame_id="/map")
        testMarks.markers[-1].ns = "circles"
        id += 1  # keep the ids unique

    # publish the marker array to be displayed on Rviz
    ankleMarkers.publish(testMarks)

    print("****************************************************************************")


if __name__ == '__main__':
    rospy.init_node('people_detection')
    combine_lasers()
