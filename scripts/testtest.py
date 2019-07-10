#!/usr/bin/env python
import rospy
import math
import copy
# from rtree import index
import matplotlib.pyplot as pp
import tf
import laser_geometry as lp
import numpy
from sensor_msgs.msg import LaserScan, PointCloud2, PointCloud, ChannelFloat32
import sensor_msgs.point_cloud2 as pc
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import random
#
# from openpose_ros_msgs.msg import *
# import bgsubtract as BG
ankleMarks = []
adjusted_laser = []
marks = MarkerArray()
laserinput = []
lastLaser = []
tf_listen = []
# humanPoses = OpenPoseHumanList()
cameraOutSize = (1280, 720)
camPixPerDeg = 16.5
tfDist_cam_laser = 0.0
tfWait = 3
show_time = 1
ind_list = []
r = .14
center = (0, 0)
center_index = 0
center_angle = 0
rate = .2
error = 0.001
circle_threshold = 15
center_points = []  # stores the centers of each detected circle
laserSettings = {}
max_range = 25.0
center_mass_points = (0, 1, 2, 5, 8, 9, 10, 11, 12, 13, 16, 17)
camAngleOff = -7.807827289 / 2  # only for test ros bag info, must be changed for tf calibrated cameras on live data
lAnkle = (0, 0)
rAnkle = (0, 0)
pc_li = PointCloud()
pc_display = PointCloud()
isFinshed = True
rand_color = [(random.randint(0, 254) / 255.0, random.randint(0, 254) / 255.0, random.randint(0, 254) / 255.0) for a in range(0,200)]


def ind_angle(b_angle):
    # print("bangle lansermin",b_angle,laserSettings["angle_min"])
    return int((b_angle - laserSettings["angle_min"]) / laserSettings["angle_increment"])


# takes in the laser scan and finds a cluster of laser points that could be a circle
# returns a list of indexes (each representing the angle from the laser) and the possible center of the circle
# important to note that this center will change with gradient descent in another function (not the true center)
def getCluster(laserScan, beginInd, endInd):
    # print("get cluster starts")
    # print(len(pc_li))
    # print("before adjust",beginInd,endInd)
    # cmake sure endInd is after beginInd
    stopInd = endInd
    startInd = beginInd
    if stopInd < startInd:
        tmp = stopInd
        stopInd = startInd
        startInd = tmp
    # print(len(laserScan.ranges))
    # make sure range is in bounds
    if startInd < 0:
        startInd = 0
    if stopInd > len(pc_li) - 1:
        stopInd = len(pc_li) - 1

    # list of indexes that are in the cluster
    list_inds = []
    # init the midpoint (will be the first point)
    test_midpoint = pc_li[startInd]
    # startPoint = pc_temp[startInd]
    # FOR TESTING
    midPointArr = []
    midpoint = (0,0,0)

    # go through the points on the laser scan to see how big the group will get
    for cind in range(startInd + 1, stopInd):

        ckind = cind - startInd

        # the point being checked
        testPoint = (pc_li[cind])

        # finding the midpoint between the tested point and the mid point
        # print(midPoint)

        test_midpoint = (
            (ckind * test_midpoint[0] + testPoint[0]) / (ckind + 1), (ckind * test_midpoint[1] + testPoint[1]) / (ckind + 1),
            (ckind * test_midpoint[2] + testPoint[2]) / (ckind + 1))


        # extend the group to the next point if the tested point is within the length of the radius to the midpoint
        # previous point is within a promxity to the next point

        proximity = .039 # arc = 2 * max_range (10) * sin ( angle/2)

        #dist_from_center(testPoint, test_midpoint) < r and
        if dist_from_center(pc_li[cind-1], pc_li[cind]) <= proximity:
            midpoint = test_midpoint
            midPointArr.append(test_midpoint)
            list_inds.append(cind)

        # if the distance between the tested point and the middle is bigger than the radius (plus error) then the group ends
        else:
            # print("midpoint" + str(midPoint))

            break
    # print("returns get cluster")
    # print(len(pc_li))
    # return the list of indexes in the group and the purposed midpoint of the possible circle (will be changed later with gradient descent)
    return list_inds, midpoint, midPointArr

#
# def convert_point(point):  # converts from a Point to a list
#     p = (point.x, point.y, point.z)
#     return p


def dist_from_center(point, center):
    square_dist = (point[0] - center[0]) ** 2 + (point[1] - center[1]) ** 2
    return square_dist ** (0.5)


# update_center will find the true center of the possible circle using gradient descent
def update_center(points):
    # print("start update center")
    # print(len(pc_li))
    global center, r, isFinshed

    grad_center_arr = []

    # isFinshed = False
    if len(points) > 0:
        new_center = center
        grad_center_arr.append(new_center)
        gradient = (1, 1)

        # print(points)
        while dist_from_center((0, 0), gradient) > 0.001:
            gradient = (0, 0)
            for pt in points:
                r_prime = dist_from_center(pt, new_center)
                dldk = (r - r_prime) * ((pt[1] - new_center[1]) / r_prime)
                dldh = (r - r_prime) * ((pt[0] - new_center[0]) / r_prime)
                gradient = (gradient[0] + dldh, gradient[1] + dldk)
            gradient = (gradient[0] / len(points), gradient[1] / len(points))
            new_center = (new_center[0] - rate * gradient[0], new_center[1] - rate * gradient[1])
            grad_center_arr.append(new_center)
        # print("center, new center", center, new_center)
        # print("end update center")
        # print(len(pc_li))
        isFinshed = True
        # print(new_center)
        return new_center, grad_center_arr
    else:
        isFinshed = True
        return

def make_PC_from_Laser(laser_in):
    cloud = []

    for x in range(len(laser_in.ranges)):
        if laser_in.ranges[x] >= laserSettings["range_max"] or laser_in.ranges[x] <= laserSettings["range_min"]:
            cloud.append((laserSettings["range_max"] * math.cos(
                laserSettings["angle_min"] + x * laserSettings["angle_increment"]),
                          laserSettings["range_max"] * math.sin(
                              laserSettings["angle_min"] + x * laserSettings["angle_increment"]), 0))
        else:
            cloud.append((laser_in.ranges[x] * math.cos(
                laserSettings["angle_min"] + x * laserSettings["angle_increment"]), laser_in.ranges[x] * math.sin(
                laserSettings["angle_min"] + x * laserSettings["angle_increment"]), 0))
    return cloud

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
    # print("end make pc from laser")
    # print(len(pc_li))
    # Returns the formatted point cloud
    # print(pc_display)
    return pc_out

def updateLaser(data):
    global lastLaser, pc_li, isFinshed, pc_display
    # print("IS FINISEDH?" + str(isFinshed))
    if isFinshed:
        # print("START UPDATE LASER " + str(len(pc_li)))pc_display
        lastLaser = copy.deepcopy(data)
        # print("updateLaser")
        if "angle_increment" not in laserSettings:
            laserSettings["angle_min"] = data.angle_min
            laserSettings["angle_max"] = data.angle_max
            laserSettings["array_size"] = len(data.ranges)
            laserSettings["angle_increment"] = data.angle_increment
            laserSettings["range_min"] = data.range_min
            laserSettings["range_max"] = data.range_max
            # build_bg(data)
        # Convert the inputted LaserScan into a Point Cloud
        # print ("FINSIEHD UPDATE LASER")
        # print(len(pc_li))
        pc_li = make_PC_from_Laser(data)
        pc_display = make_PC_from_Laser_display(data)

def showCircles():
    global ankleMarks, bg_pub, tf_listen
    global ind_list, center, center_index
    global laserList, laserSettings, isFinshed, center_points, pc_li, rand_color

    # print("start")

    # initialize the node for ROS
    rospy.init_node("ankle_markers")

    # initialize the subscribers and the publishers for the laser, point cloud, tf, and the markers
    laserList = rospy.Subscriber("/hog/scan0", LaserScan, updateLaser)
    circleMarks = rospy.Publisher("/possible_circles", MarkerArray, queue_size=10)
    bg_pub = rospy.Publisher('/bg_cloud', PointCloud, queue_size=10)
    tf_listen = tf.TransformListener(True, rospy.Duration(1.0))

    # Set the frame for Point Cloud in Rviz
    pc_li.header.frame_id = "bg_cloud"


    while not rospy.is_shutdown():

        # the MarkerArray will store all the circles to be put on Rviz
        testMarks = MarkerArray()

        # if there is a laser scan being read in
        if lastLaser:

            # update the laser scan (this line may not be needed)
            updateLaser(lastLaser)

            isFinshed = False
            center_points = []
            # convert the new laser scan into a point cloud
            # pcloud = make_PC_from_Laser(lastLaser)

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
            while center_index < len(pc_li):

                # list of indexes making up the cluster
                ind_list = []


                # finding where to start the cluster (if the point cloud is not inf)
                # if not math.isinf(lastLaser.ranges[center_index]):
                #  obtaining the list of indexes making up the cluster and the possible center of the circle
                ind_list, midrange, midPointArr1 = getCluster(lastLaser, center_index, len(
                    pc_li))  # search to the whole scan until the shape is gone

                # points needed to be considered a circle threshold
                if len(ind_list) > circle_threshold:

                    # FOR TESTING: Marking each cluster with a different color
                    # rand_color = (random.randint(0,254)/255.0, random.randint(0,254)/255.0, random.randint(0,254)/255.0)
                    #random.uniform(0,1), random.uniform(0,1), random.uniform(0,1), 1)
                    # print(rand_color)
                    for ind in ind_list:

                        rgba = ColorRGBA(.23, .62, .67, 1)

                        # add a new marker to the marker array
                        testMarks.markers.append(Marker())

                        # keep the marker ids unique
                        testMarks.markers[-1].id = id

                        # determining how long the markers will stay up in Rviz
                        testMarks.markers[-1].lifetime = rospy.Duration(show_time)

                        # find the tf of the laser to the parent map
                        # this tf will be used to translate the circles to match the laser's map
                        try:
                            (trans, rot) = tf_listen.lookupTransform('/map', '/laser_hog', rospy.Time(0))
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            continue

                        # postioning will be relative to the tf of the laser
                        testMarks.markers[-1].pose = Pose(
                            # Point(updated_center[0] + trans[0], updated_center[1] + trans[1], 0 + trans[2]),
                            Point(pc_li[ind][0], pc_li[ind][1], 0),
                            Quaternion(0, 0, 0, 1))

                        testMarks.markers[-1].type = Marker.SPHERE
                        testMarks.markers[-1].scale = Vector3(.03, .03, .03)
                        testMarks.markers[-1].action = 0
                        testMarks.markers[-1].color = ColorRGBA(rand_color[group_index % 200][0], rand_color[group_index % 200][1], rand_color[group_index % 200][2] ,1)
                        testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
                        testMarks.markers[-1].ns = "clusters"
                        id += 1  # keep the ids unique


                    # if the shape can be considered to be a circle, then add the midpoint (key) and the index list making up the portion of the circle
                    clusters.append((midrange, ind_list))

                    # print("midpoints")
                    # print(midPointArr1)

                    # print(clusters[-1])
                    # FOR TESTING
                    # checking where the midpoints are for each possible circle
                    for mp in midPointArr1:

                        # print(mp)
                        # add a new marker to the marker array
                        testMarks.markers.append(Marker())

                        # keep the marker ids unique
                        testMarks.markers[-1].id = id

                        # determining how long the markers will stay up in Rviz
                        testMarks.markers[-1].lifetime = rospy.Duration(show_time)

                        # find the tf of the laser to the parent map
                        # this tf will be used to translate the circles to match the laser's map
                        try:
                            (trans, rot) = tf_listen.lookupTransform('/map', '/laser_hog', rospy.Time(0))
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            continue

                        # postioning will be relative to the tf of the laser
                        testMarks.markers[-1].pose = Pose(
                            # Point(updated_center[0] + trans[0], updated_center[1] + trans[1], 0 + trans[2]),
                            Point(mp[0], mp[1], 0),
                            Quaternion(0, 0, 0, 1))

                        testMarks.markers[-1].type = Marker.SPHERE
                        testMarks.markers[-1].scale = Vector3(.005, .005, .01)
                        testMarks.markers[-1].action = 0
                        testMarks.markers[-1].color = ColorRGBA(.2, .9, .9, 1)
                        testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
                        testMarks.markers[-1].ns = "midpoints"
                        id += 1  # keep the ids unique

                    # FOR TESTING: marking the last midpoint
                    testMarks.markers.append(Marker())
                    testMarks.markers[-1].id = id

                    testMarks.markers[-1].pose = Pose(
                        # Point(updated_center[0] + trans[0], updated_center[1] + trans[1], 0 + trans[2]),
                        Point(midPointArr1[-1][0], midPointArr1[-1][1], 0),
                        Quaternion(0, 0, 0, 1))
                    testMarks.markers[-1].lifetime = rospy.Duration(show_time)

                    testMarks.markers[-1].type = Marker.SPHERE
                    testMarks.markers[-1].scale = Vector3(.01, .01, .01)
                    testMarks.markers[-1].action = 0
                    testMarks.markers[-1].color = ColorRGBA(0, .9 , .5, 1)
                    testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
                    testMarks.markers[-1].ns = "last_midpoint"
                    id += 1  # keep the ids unique

                    # testMarks.markers.append(Marker())
                    # testMarks.markers[-1].id = id
                    #
                    # testMarks.markers[-1].pose = Pose(
                    #     # Point(updated_center[0] + trans[0], updated_center[1] + trans[1], 0 + trans[2]),
                    #     Point(1, 1, 0),
                    #     Quaternion(0, 0, 0, 1))
                    # testMarks.markers[-1].lifetime = rospy.Duration(show_time)
                    #
                    # testMarks.markers[-1].type = Marker.SPHERE
                    # testMarks.markers[-1].scale = Vector3(1, 1, 0)
                    # testMarks.markers[-1].action = 0
                    # testMarks.markers[-1].color = ColorRGBA(0, .9 , .5, .5)
                    # testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
                    # testMarks.markers[-1].ns = "random"
                    # id += 1

                    point_for_update = []

                    for i in ind_list:

                        # add the x,y,z rectangular point into the array for use in update_center
                        point_for_update.append((pc_li[i]))

                    # print(point_for_update)

                    center = midPointArr1[-1]

                    # updated center with the given points in this cluster
                    updated_center, grad_center_arr1 = update_center(point_for_update)

                    temp_r = dist_from_center(point_for_update[0], updated_center)

                    # print(str(updated_center) + " " + str(dist_from_center(point_for_update[0], updated_center)))

                    # set a minimum range away from the laser to detect circles
                    # this will remove alot of the false positives we are getting that are close to the laser
                    # (assume that targets are not within 1 meter)
                    if dist_from_center((0,0), updated_center) >= 1.0:

                        # add the updated center to a list of circles (used later on in triangle_finder)
                        center_points.append(updated_center)

                        # add a new marker to the marker array
                        testMarks.markers.append(Marker())

                        # keep the marker ids unique
                        testMarks.markers[-1].id = id

                        # determining how long the markers will stay up in Rviz
                        testMarks.markers[-1].lifetime = rospy.Duration(show_time)

                        # find the tf of the laser to the parent map
                        # this tf will be used to translate the circles to match the laser's map
                        # try:
                        #     (trans, rot) = tf_listen.lookupTransform('/laser_hog', '/map', rospy.Time(0))
                        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        #     continue

                        # postioning will be relative to the tf of the laser
                        testMarks.markers[-1].pose = Pose(
                            # Point(updated_center[0] + trans[0], updated_center[1] + trans[1], 0 + trans[2]),
                            Point(updated_center[0], updated_center[1], 0),
                            Quaternion(0, 0, 0, 1))

                        testMarks.markers[-1].type = Marker.SPHERE
                        testMarks.markers[-1].scale = Vector3(2* r, 2 * r, .01)
                        testMarks.markers[-1].action = 0
                        testMarks.markers[-1].color = ColorRGBA(.0, .8, .27, 1)
                        testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
                        testMarks.markers[-1].ns = "circles"
                        id += 1  # keep the ids unique

                        # FOR TESTING
                        # Display the grad descent process
                        for pt in grad_center_arr1:

                            # add a new marker to the marker array
                            testMarks.markers.append(Marker())

                            # keep the marker ids unique
                            testMarks.markers[-1].id = id

                            # determining how long the markers will stay up in Rviz
                            testMarks.markers[-1].lifetime = rospy.Duration(show_time)

                            # postioning will be relative to the tf of the laser
                            testMarks.markers[-1].pose = Pose(
                                # Point(updated_center[0] + trans[0], updated_center[1] + trans[1], 0 + trans[2]),
                                Point(pt[0], pt[1], 0),
                                Quaternion(0, 0, 0, 1))
                            testMarks.markers[-1].type = Marker.SPHERE
                            testMarks.markers[-1].scale = Vector3(.005, .005, .01)
                            testMarks.markers[-1].action = 0
                            testMarks.markers[-1].color = ColorRGBA(.1, .2, .1, 1)
                            testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
                            testMarks.markers[-1].ns = "grad_descent"
                            id += 1  # keep the ids unique

                group_index += 1

                # iterate to the next laser point available
                center_index += len(ind_list) + 1

                # else:  # if the certain index at this angle is inf, don't bother running through the algorithm
                # iterate to the next laser point available
                # center_index += 1

            # publish the marker array to be displayed on Rviz
            circleMarks.publish(testMarks)

            # Publish the Point Cloud data
            # print(pc_li)
            bg_pub.publish(pc_display)

            triangle_finder()
            isFinshed = True
            print("****************************************************************************")



# finds the triangle made by three circles
def triangle_finder():
    global center_points

    triangle_marks = rospy.Publisher("/triangles", MarkerArray, queue_size=10)

    # measurements of physical triangle (calibration target)
    leg_a = .48
    leg_b = .62
    leg_c = .78
    angle_a = math.sin(leg_a / leg_c)
    angle_b = math.sin(leg_b / leg_c)
    angle_c = math.pi / 2
    margin = .05

    # for testing
    # center_points = [(0, 0), (0.3, 0), (0, 0.4), (.4, .3)]

    len_measurment_arr = []
    possible_legs_a = []
    possible_legs_b = []
    possible_legs_c = []

    possible_legs_matrix = [[0 for x in range(len(center_points))] for y in range(len(center_points))]
    print(len(center_points))
    for i in range(0, len(center_points)):
        for j in range(i, len(center_points)):
            if i != j:
                # finding possible legs by using the lengths given between each circle
                temp_dist = dist_from_center(center_points[i], center_points[j])

                # checking to see if the lengths between two circles could be one of the legs
                if leg_a - margin < temp_dist < leg_a + margin:
                    len_measurment_arr.append([center_points[i], center_points[j], 'leg a'])
                    possible_legs_matrix[i][j] = 'a'
                    possible_legs_a.append([center_points[i], center_points[j]])
                elif leg_b - margin < temp_dist < leg_b + margin:
                    len_measurment_arr.append([center_points[i], center_points[j], 'leg b'])
                    possible_legs_matrix[i][j] = 'b'
                    possible_legs_b.append([center_points[i], center_points[j]])
                elif leg_c - margin < temp_dist < leg_c + margin:
                    len_measurment_arr.append([center_points[i], center_points[j], 'leg c'])
                    possible_legs_matrix[i][j] = 'c'
                    possible_legs_c.append([center_points[i], center_points[j]])

    possible_triangle = []
    testMarks = MarkerArray()
    id = 0

    for leg_points in possible_legs_a:
        # add a new marker to the marker array
        testMarks.markers.append(Marker())

        # keep the marker ids unique
        testMarks.markers[-1].id = id

        # determining how long the markers will stay up in Rviz
        testMarks.markers[-1].lifetime = rospy.Duration(show_time)
        testMarks.markers[-1].type = Marker.LINE_STRIP
        testMarks.markers[-1].scale = Vector3(.03, .03, .01)
        testMarks.markers[-1].action = 0
        testMarks.markers[-1].color = ColorRGBA(0,0,1,1)
        testMarks.markers[-1].points = [Point(leg_points[0][0], leg_points[0][1], 0), Point(leg_points[1][0], leg_points[1][1], 0)]
        testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
        testMarks.markers[-1].ns = "legs_a"
        id += 1  # keep the ids unique

    for leg_points in possible_legs_b:
        # add a new marker to the marker array
        testMarks.markers.append(Marker())

        # keep the marker ids unique
        testMarks.markers[-1].id = id

        # determining how long the markers will stay up in Rviz
        testMarks.markers[-1].lifetime = rospy.Duration(show_time)
        testMarks.markers[-1].type = Marker.LINE_STRIP
        testMarks.markers[-1].scale = Vector3(.03, .03, .01)
        testMarks.markers[-1].action = 0
        testMarks.markers[-1].color = ColorRGBA(0,.9,.4,1)
        testMarks.markers[-1].points = [Point(leg_points[0][0], leg_points[0][1], 0), Point(leg_points[1][0], leg_points[1][1], 0)]
        testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
        testMarks.markers[-1].ns = "legs_b"
        id += 1  # keep the ids unique

    for leg_points in possible_legs_c:
        # add a new marker to the marker array
        testMarks.markers.append(Marker())

        # keep the marker ids unique
        testMarks.markers[-1].id = id

        # determining how long the markers will stay up in Rviz
        testMarks.markers[-1].lifetime = rospy.Duration(show_time)
        testMarks.markers[-1].type = Marker.LINE_STRIP
        testMarks.markers[-1].scale = Vector3(.03, .03, .01)
        testMarks.markers[-1].action = 0
        testMarks.markers[-1].color = ColorRGBA(.2, .8, .9, 1)
        testMarks.markers[-1].points = [Point(leg_points[0][0], leg_points[0][1], 0),
                                        Point(leg_points[1][0], leg_points[1][1], 0)]
        testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
        testMarks.markers[-1].ns = "legs_c"
        id += 1  # keep the ids unique


    print(len(possible_legs_a))
    print(len(possible_legs_b))
    print(len(possible_legs_c))

    # find the angle between the lines to determine which could be in the tirangle
    # efficiency of n^3, probably a way to make this more efficient
    # if there is a chance to find a triangle (possible if there is at least one leg)
    if (len(possible_legs_c) > 0 and len(possible_legs_b) > 0 and len(possible_legs_a) > 0):
        for a in possible_legs_a:
            for b in possible_legs_b:
                for c in possible_legs_c:
                    a_dist = dist_from_center(a[0], a[1])
                    b_dist = dist_from_center(b[0], b[1])
                    c_dist = dist_from_center(c[0], c[1])
                    # print(str(a_dist) + " " + str(b_dist) + " " + str(c_dist))
                    angle_range = 5.0 / 180.0 * math.pi

                    if (angle_a - angle_range < math.sin(a_dist / c_dist) < angle_a + angle_range) and (angle_b - angle_range < math.sin(b_dist / c_dist) < angle_b + angle_range):
                        possible_triangle.append([a, b, c])
    #
    # print("poss")
    # print(possible_triangle)
    #
    # for tri in possible_triangle:
    #     triangle_points = []
    #     print("tri")
    #     print(tri)
    #     for pt in tri:
    #         print("pt")
    #         print(pt)
    #         for ptp in pt:
    #             if ptp not in triangle_points:
    #                 triangle_points.append(ptp)
    #
    #     print("triangle")
    #     print(triangle_points)
    #     # add a new marker to the marker array
    #     testMarks.markers.append(Marker())
    #
    #     # keep the marker ids unique
    #     testMarks.markers[-1].id = id
    #
    #     # determining how long the markers will stay up in Rviz
    #     testMarks.markers[-1].lifetime = rospy.Duration(show_time)
    #     testMarks.markers[-1].type = Marker.TRIANGLE_LIST
    #     testMarks.markers[-1].scale = Vector3(1, 1, 1)
    #     testMarks.markers[-1].action = 0
    #     testMarks.markers[-1].color = ColorRGBA(.7,.7,.2, 1)
    #     testMarks.markers[-1].points = [Point(triangle_points[0][0], triangle_points[0][1], 0),
    #                                     Point(triangle_points[1][0], triangle_points[1][1], 0),
    #                                     Point(triangle_points[2][0], triangle_points[2][1], 0)]
    #     testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
    #     testMarks.markers[-1].ns = "triangle"
    #     id += 1  # keep the ids unique

    triangle_marks.publish(testMarks)

    # for i in range(0, len(center_points)):
    #     for j in range(i, len(center_points)):
    #         if i != j:
    #             if possible_legs_matrix[i][j] = 'a':
    #                 for x in range(i, len(center_points)):
    #                     if possible_legs_matrix[x][j] = 'b':
    #                         for y in range(j, len(center_points)):
    #                             if possible_legs_matrix[x][y] = 'c':



    # exit()

if __name__ == '__main__':
    showCircles()
