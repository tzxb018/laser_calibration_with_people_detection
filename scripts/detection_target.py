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
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3


# marks = MarkerArray()
laserinput = []
lastLaser = []
show_time = 0
ind_list = []
r = float(sys.argv[4])
center = (0, 0)
center_index = 0
center_angle = 0
rate = .2
error = 0.001
circle_threshold = 8
center_points = []  # stores the centers of each detected circle
possible_triangle = [] # stores the point that make up the calibration target
laserSettings = {}
max_range = 25.0
pc_li = PointCloud()
pc_display = PointCloud()
laser_pub = []
# isFinshed = True
laser_in = LaserScan()
send_matrix = []
laser_topic_name = ""
max_count = 20
min_dist = .25
timeout_time = rospy.Duration(10.0)

def detection_target(req):
    global laser_in, pc_li, pc_display, max_count, timeout_time
    # print(req.laser_topic)
    a = 0
    # goes through the laser 10 times to try to find the calibration target for each laser
    while True:
        a += 1
        # getting the input laser scan for the service
        print("waiting for rospy message from " + str(req.laser_topic) + "...")
        laser_in = rospy.wait_for_message(req.laser_topic, LaserScan)
        print("retrived rospy message from " + str(req.laser_topic))

        # converting the laser scan into a point cloud for easier calculations
        updateLaser(laser_in)

        out = showCircles()

        # if the target is found, break the searching while loop
        if out != []:
            print("out")
            print("Point a\n" + str(Point(out[6], out[7], 0)))
            print("Point c\n" + str(Point(out[4], out[5], 0)))
            print("Point b\n" + str(Point(out[8], out[9], 0)))
            print("****************************************************************************")
            break

        if a >= max_count:
            print("all attempts failed on %s"%(req.laser_topic))
            break

        print("attempt %d failed. Try again."%(a))

    # returns the info for the service detection_target.srv
    # geometry_msgs/Point point_a, geometry_msgs/Point point_c
    return Point(out[6], out[7], 0), Point(out[4], out[5], 0), Point(out[8], out[9], 0)


# takes in the laser scan and finds a cluster of laser points that could be a circle
# returns a list of indexes (each representing the angle from the laser) and the possible center of the circle
# important to note that this center will change with gradient descent in another function (not the true center)
def getCluster(laserScan, beginInd, endInd):
    stopInd = endInd
    startInd = beginInd
    if stopInd < startInd:
        tmp = stopInd
        stopInd = startInd
        startInd = tmp

    # make sure range is in bounds
    if startInd < 0:
        startInd = 0
    if stopInd > len(pc_li) - 1:
        stopInd = len(pc_li) - 1

    # list of indexes that are in the cluster
    list_inds = []

    # init the midpoint (will be the first point)
    test_midpoint = pc_li[startInd]

    # FOR TESTING
    midPointArr = []
    midpoint = (0, 0, 0)

    # go through the points on the laser scan to see how big the group will get
    for cind in range(startInd + 1, stopInd):

        ckind = cind - startInd

        # the point being checked
        testPoint = (pc_li[cind])

        # finding the midpoint between the tested point and the mid point
        # print(midPoint)

        test_midpoint = (
            (ckind * test_midpoint[0] + testPoint[0]) / (ckind + 1),
            (ckind * test_midpoint[1] + testPoint[1]) / (ckind + 1),
            (ckind * test_midpoint[2] + testPoint[2]) / (ckind + 1))

        # extend the group to the next point if the tested point is within the length of the radius to the midpoint
        # previous point is within a close proximity to the next point

        proximity = .039  # arc = 2 * max_range (10) * sin ( angle/2)

        #
        if dist_from_center(pc_li[cind - 1], pc_li[cind]) <= proximity:
            midpoint = test_midpoint
            midPointArr.append(test_midpoint)
            list_inds.append(cind)

        # if the distance between the tested point and the middle is bigger than the radius (plus error) then the group ends
        else:
            break

    # return the list of indexes in the group and the purposed midpoint of the possible circle (will be changed later with gradient descent)
    return list_inds, midpoint, midPointArr


def dist_from_center(point, center):
    square_dist = (point[0] - center[0]) ** 2 + (point[1] - center[1]) ** 2
    return square_dist ** 0.5

# update_center will find the true center of the possible circle using gradient descent
def update_center(points):
    global center, r, isFinshed

    grad_center_arr = []

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

        # isFinshed = True
        return new_center, grad_center_arr
    else:
        # isFinshed = True
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
    return pc_out

def updateLaser(data):
    global lastLaser, pc_li, isFinshed, pc_display, laser_pub
    #
    # if isFinshed:
    lastLaser = copy.deepcopy(data)

    if "angle_increment" not in laserSettings:
        laserSettings["angle_min"] = data.angle_min
        laserSettings["angle_max"] = data.angle_max
        laserSettings["array_size"] = len(data.ranges)
        laserSettings["angle_increment"] = data.angle_increment
        laserSettings["range_min"] = data.range_min
        laserSettings["range_max"] = data.range_max

    pc_li = make_PC_from_Laser(data)
    pc_display = make_PC_from_Laser_display(data)
    # print(data.header.stamp)
    lastLaser.header.stamp = rospy.Time.now()
    # laser_pub.publish(lastLaser)

def showCircles():
    global ankleMarks, bg_pub, tf_listen
    global ind_list, center, center_index
    global laserList, laserSettings, isFinshed, center_points, pc_li, rand_color, laser_pub, min_dist

    global laser_in

    # initialize the subscribers and the publishers for the laser, point cloud, tf, and the markers
    # laser_pub = rospy.Publisher("/hog/updatedScan", LaserScan, queue_size = 10)


    # Set the frame for Point Cloud in Rviz
    # pc_li.header.frame_id = "/hog/bg_cloud"

    # the MarkerArray will store all the circles to be put on Rviz
    # testMarks = MarkerArray()

    lastLaser = laser_in

    # if there is a laser scan being read in
    if lastLaser:

        # isFinshed = False
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
                # random.uniform(0,1), random.uniform(0,1), random.uniform(0,1), 1)
                # print(rand_color)
                # for ind in ind_list:
                #
                #     rgba = ColorRGBA(.23, .62, .67, 1)
                #
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
                #         Point(pc_li[ind][0], pc_li[ind][1], 0),
                #         Quaternion(0, 0, 0, 1))
                #
                #     testMarks.markers[-1].type = Marker.SPHERE
                #     testMarks.markers[-1].scale = Vector3(.03, .03, .03)
                #     testMarks.markers[-1].action = 0
                #     testMarks.markers[-1].color = ColorRGBA(rand_color[group_index % 200][0],
                #                                             rand_color[group_index % 200][1],
                #                                             rand_color[group_index % 200][2], 1)
                #     testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
                #     testMarks.markers[-1].ns = "clusters"
                #     id += 1  # keep the ids unique

                # if the shape can be considered to be a circle, then add the midpoint (key) and the index list making up the portion of the circle
                clusters.append((midrange, ind_list))

                # print("midpoints")
                # print(midPointArr1)

                # print(clusters[-1])
                # FOR TESTING
                # checking where the midpoints are for each possible circle
                # for mp in midPointArr1:
                #
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
                #     # postioning will be relative to the tf of the laser
                #     testMarks.markers[-1].pose = Pose(
                #         # Point(updated_center[0] + trans[0], updated_center[1] + trans[1], 0 + trans[2]),
                #         Point(mp[0], mp[1], 0),
                #         Quaternion(0, 0, 0, 1))
                #
                #     testMarks.markers[-1].type = Marker.SPHERE
                #     testMarks.markers[-1].scale = Vector3(.005, .005, .01)
                #     testMarks.markers[-1].action = 0
                #     testMarks.markers[-1].color = ColorRGBA(.2, .9, .9, 1)
                #     testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
                #     testMarks.markers[-1].ns = "midpoints"
                #     id += 1  # keep the ids unique

                # FOR TESTING: marking the last midpoint
                # testMarks.markers.append(Marker())
                # testMarks.markers[-1].id = id
                #
                # testMarks.markers[-1].pose = Pose(
                #     # Point(updated_center[0] + trans[0], updated_center[1] + trans[1], 0 + trans[2]),
                #     Point(midPointArr1[-1][0], midPointArr1[-1][1], 0),
                #     Quaternion(0, 0, 0, 1))
                # testMarks.markers[-1].lifetime = rospy.Duration(show_time)
                #
                # testMarks.markers[-1].type = Marker.SPHERE
                # testMarks.markers[-1].scale = Vector3(.01, .01, .01)
                # testMarks.markers[-1].action = 0
                # testMarks.markers[-1].color = ColorRGBA(0, .9, .5, 1)
                # testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
                # testMarks.markers[-1].ns = "last_midpoint"
                # id += 1  # keep the ids unique

                point_for_update = []

                for i in ind_list:
                    # add the x,y,z rectangular point into the array for use in update_center
                    point_for_update.append((pc_li[i]))

                # print(point_for_update)

                center = midPointArr1[-1]

                # updated center with the given points in this cluster
                updated_center, grad_center_arr1 = update_center(point_for_update)

                within_circle = True
                within_margin = .03
                for i in ind_list:
                    if not (r - within_margin <= dist_from_center(pc_li[i], updated_center) <= r + within_margin):
                        within_circle = False

                # FOR TESTING: drawing the margin circles
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
                #     Point(updated_center[0], updated_center[1], 0),
                #     Quaternion(0, 0, 0, 1))
                #
                # testMarks.markers[-1].type = Marker.SPHERE
                # testMarks.markers[-1].scale = Vector3(2 * r + within_margin, 2 * r + within_margin, .01)
                # testMarks.markers[-1].action = 0
                # testMarks.markers[-1].color = ColorRGBA(.0, .9, .0, .1)
                # testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
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
                #     Point(updated_center[0], updated_center[1], 0),
                #     Quaternion(0, 0, 0, 1))
                #
                # testMarks.markers[-1].type = Marker.SPHERE
                # testMarks.markers[-1].scale = Vector3(2 * r - within_margin, 2 * r - within_margin, .01)
                # testMarks.markers[-1].action = 0
                # testMarks.markers[-1].color = ColorRGBA(.0, .9, .0, .1)
                # testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
                # testMarks.markers[-1].ns = "margin"
                # id += 1  # keep the ids unique
                id += 1
                if within_circle:

                    # set a minimum range away from the laser to detect circles
                    # this will remove alot of the false positives we are getting that are close to the laser
                    # (assume that targets are not within 1 meter)
                    # if abs(updated_center[0]) >= 1 or abs(updated_center[1]) >= 1:
                    if dist_from_center(updated_center, (0,0)) >= min_dist:
                        # add the updated center to a list of circles (used later on in triangle_finder)
                        center_points.append(updated_center + (id,))

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
                        #     Point(updated_center[0], updated_center[1], 0),
                        #     Quaternion(0, 0, 0, 1))
                        #
                        # testMarks.markers[-1].type = Marker.SPHERE
                        # testMarks.markers[-1].scale = Vector3(2 * r, 2 * r, .01)
                        # testMarks.markers[-1].action = 0
                        # testMarks.markers[-1].color = ColorRGBA(.0, .8, .27, .1)
                        # testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
                        # testMarks.markers[-1].ns = "circles"
                        # id += 1  # keep the ids unique

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
                        #         Point(pt[0], pt[1], 0),
                        #         Quaternion(0, 0, 0, 1))
                        #     testMarks.markers[-1].type = Marker.SPHERE
                        #     testMarks.markers[-1].scale = Vector3(.005, .005, .01)
                        #     testMarks.markers[-1].action = 0
                        #     testMarks.markers[-1].color = ColorRGBA(.8, .6, .6, 1)
                        #     testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
                        #     testMarks.markers[-1].ns = "grad_descent"
                        #     id += 1  # keep the ids unique

            group_index += 1

            # iterate to the next laser point available
            center_index += len(ind_list) + 1

        # publish the marker array to be displayed on Rviz
        # circleMarks.publish(testMarks)

        # Publish the Point Cloud data
        # bg_pub.publish(pc_display)

        # Finds the calibration triangle using the circles detected before
        out = triangle_finder()

        # isFinshed = True
        return out

# finds the triangle made by three circles
def triangle_finder():
    global center_points, possible_triangle


    # measurements of physical triangle (calibration target)
    # leg c is the hypotenuse
    leg_a = float(sys.argv[1])
    leg_b = float(sys.argv[2])
    leg_c = float(sys.argv[3])
    margin = .05

    # for storing all possible legs
    len_measurment_arr = []
    possible_legs_a = []
    possible_legs_b = []
    possible_legs_c = []

    # possible_legs_matrix = [[0 for x in range(len(center_points))] for y in range(len(center_points))]

    # print(len(center_points))
    for i in range(0, len(center_points)):
        for j in range(i, len(center_points)):
            if i != j:
                # finding possible legs by using the lengths given between each circle
                temp_dist = dist_from_center(center_points[i], center_points[j])

                # checking to see if the lengths between two circles could be one of the legs
                if leg_a - margin < temp_dist < leg_a + margin:
                    len_measurment_arr.append([center_points[i], center_points[j], 'leg a'])
                    # possible_legs_matrix[i][j] = 'a'
                    possible_legs_a.append([center_points[i], center_points[j]])
                elif leg_b - margin < temp_dist < leg_b + margin:
                    len_measurment_arr.append([center_points[i], center_points[j], 'leg b'])
                    # possible_legs_matrix[i][j] = 'b'
                    possible_legs_b.append([center_points[i], center_points[j]])
                elif leg_c - margin < temp_dist < leg_c + margin:
                    len_measurment_arr.append([center_points[i], center_points[j], 'leg c'])
                    # possible_legs_matrix[i][j] = 'c'
                    possible_legs_c.append([center_points[i], center_points[j]])

    # print(possible_legs_matrix)
    possible_triangle = []


    # for leg_points in possible_legs_a:
    #     # add a new marker to the marker array
    #     testMarks.markers.append(Marker())
    #
    #     # keep the marker ids unique
    #     testMarks.markers[-1].id = id
    # are
    #     # determining how long the markers will stay up in Rviz
    #     testMarks.markers[-1].lifetime = rospy.Duration(show_time)
    #     testMarks.markers[-1].type = Marker.LINE_STRIP
    #     testMarks.markers[-1].scale = Vector3(.03, .03, .01)
    #     testMarks.markers[-1].action = 0
    #     testMarks.markers[-1].color = ColorRGBA(0,0,1,1)
    #     testMarks.markers[-1].points = [Point(leg_points[0][0], leg_points[0][1], 0), Point(leg_points[1][0], leg_points[1][1], 0)]
    #     testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
    #     testMarks.markers[-1].ns = "legs_a"
    #     id += 1  # keep the ids unique
    #
    # for leg_points in possible_legs_b:
    #     # add a new marker to the marker array
    #     testMarks.markers.append(Marker())
    #
    #     # keep the marker ids unique
    #     testMarks.markers[-1].id = id
    #
    #     # determining how long the markers will stay up in Rviz
    #     testMarks.markers[-1].lifetime = rospy.Duration(show_time)
    #     testMarks.markers[-1].type = Marker.LINE_STRIP
    #     testMarks.markers[-1].scale = Vector3(.03, .03, .01)
    #     testMarks.markers[-1].action = 0
    #     testMarks.markers[-1].color = ColorRGBA(0,.9,.4,1)
    #     testMarks.markers[-1].points = [Point(leg_points[0][0], leg_points[0][1], 0), Point(leg_points[1][0], leg_points[1][1], 0)]
    #     testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
    #     testMarks.markers[-1].ns = "legs_b"
    #     id += 1  # keep the ids unique
    #
    # for leg_points in possible_legs_c:
    #     # add a new marker to the marker array
    #     testMarks.markers.append(Marker())
    #
    #     # keep the marker ids unique
    #     testMarks.markers[-1].id = id
    #
    #     # determining how long the markers will stay up in Rviz
    #     testMarks.markers[-1].lifetime = rospy.Duration(show_time)
    #     testMarks.markers[-1].type = Marker.LINE_STRIP
    #     testMarks.markers[-1].scale = Vector3(.03, .03, .01)
    #     testMarks.markers[-1].action = 0
    #     testMarks.markers[-1].color = ColorRGBA(.2, .8, .9, 1)
    #     testMarks.markers[-1].points = [Point(leg_points[0][0], leg_points[0][1], 0),
    #                                     Point(leg_points[1][0], leg_points[1][1], 0)]
    #     testMarks.markers[-1].header = lastLaser.header  # Header(frame_id="/map")
    #     testMarks.markers[-1].ns = "legs_c"
    #     id += 1  # keep the ids unique


    # print(len(possible_legs_a))
    # print(len(possible_legs_b))
    # print(len(possible_legs_c))


    # Finding the triangle using the lengths given and shared points
    # Making sure that there is at least one possibility for each leg
    if len(possible_legs_c) > 0 and len(possible_legs_b) > 0 and len(possible_legs_a) > 0:
        for a in possible_legs_a:
            for b in possible_legs_b:
                for c in possible_legs_c:

                    # storing all the centers for this combination
                    list_of_centers = []

                    # if the point is distinct, add to the array
                    if a[0][2] not in list_of_centers:
                        list_of_centers.append(a[0][2])

                    if a[1][2] not in list_of_centers:
                        list_of_centers.append(a[1][2])

                    if b[0][2] not in list_of_centers:
                        list_of_centers.append(b[0][2])

                    if b[1][2] not in list_of_centers:
                        list_of_centers.append(b[1][2])

                    if c[0][2] not in list_of_centers:
                        list_of_centers.append(c[0][2])

                    if c[1][2] not in list_of_centers:
                        list_of_centers.append(c[1][2])

                    # if there are three and only three distinct points, its our triangle
                    if len(list_of_centers) > 3:
                        continue

                    # if three distinct points, add to possible triangles
                    print("a: " + str(a))
                    print("b: " + str(b))
                    print("c: " + str(c))
                    possible_triangle.append([a, b, c])


    # print(possible_triangle)

    # Displaying the triangle on Rviz
    for triangle in possible_triangle:
        leg_num = 1
        for line_points in triangle:

            # determines the color of each leg
            if leg_num == 1:
                leg_color = ColorRGBA(.3, .7, .6, 1)
            elif leg_num == 2:
                leg_color = ColorRGBA(.6, .1, .5, 1)
            else:
                leg_color = ColorRGBA(.8, .8, .7, 1)

            leg_num += 1



    out = matrix_transformation()
    return out

# finding the matrix transformation using the triangle
def matrix_transformation():
    global possible_triangle, send_matrix

    # publisher = rospy.Publisher('/matrix_for_mouse', matrix_tf, queue_size=10)
    # rospy.init_node('hog_talker', anonymous=True)

    # basis = matrix_tf()
    # basis.header.stamp = rospy.Time.now()
    # basis.header.frame_id = "/map"

    if len(possible_triangle) > 0:
        # for triangle in possible_triangle:
        triangle = possible_triangle[0]

        print(triangle)

        # find the a and b legs (the perpendicular)
        line_a = triangle[0]
        line_b = triangle[1]

        # finds the common point between legs a and b, the right angle of the triangle
        if triangle[0][0] in triangle[1]:
            point_c = triangle[0][0]
            point_a = triangle[0][1]
        else:
            point_c = triangle[0][1]
            point_a = triangle[0][0]

        # finds point b
        if point_c == triangle[1][0]:
            point_b = triangle[1][1]
        else:
            point_b = triangle[1][0]

        # finding the basis using the legs
        grad_a_rise = float((line_a[0][1] - line_a[1][1]))
        grad_a_run = float(line_a[0][0] - line_a[1][0])
        grad_b_rise = float((line_b[0][1] - line_b[1][1]))
        grad_b_run = float(line_b[0][0] - line_b[1][0])

        send_matrix = [grad_a_rise, grad_a_run, grad_b_rise, grad_b_run, point_c[0], point_c[1], point_a[0], point_a[1], point_b[0], point_b[1]]
        return send_matrix
    else:
        send_matrix = []
        return send_matrix


def detection_target_server():
    rospy.init_node('detection_Target')
    s = rospy.Service('detection_target', Detection_target, detection_target)
    rospy.spin()

if __name__ == '__main__':
    detection_target_server()