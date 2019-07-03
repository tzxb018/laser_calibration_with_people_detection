#!/usr/bin/env python

# -*- coding: utf-8 -*-
# @Author: andrew
# @Date:   2018-07-16 14:37:41
# @Last Modified by:   ahpalmerUNR
# @Last Modified time: 2018-11-04 18:00:30

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
r = .075
center = (0, 0)
center_index = 0
center_angle = 0
rate = 0.05
error = 0.001

laserSettings = {}
max_range = 25.0
center_mass_points = (0, 1, 2, 5, 8, 9, 10, 11, 12, 13, 16, 17)

camAngleOff = -7.807827289 / 2  # only for test ros bag info, must be changed for tf calibrated cameras on live data

lAnkle = (0, 0)
rAnkle = (0, 0)


def ind_angle(b_angle):
    # print("bangle lansermin",b_angle,laserSettings["angle_min"])
    return int((b_angle - laserSettings["angle_min"]) / laserSettings["angle_increment"])

# takes in the laser scan and finds a cluster of laser points that could be a circle
# returns a list of indexes (each representing the angle from the laser) and the possible center of the circle
# important to note that this center will change with gradient descent in another function (not the true center)
def getCluster(laserScan, beginInd, endInd):
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
    if stopInd > len(laserScan.ranges) - 1:
        stopInd = len(laserScan.ranges) - 1

    # print(startInd,stopInd)

    # converting laser scan into point cloud
    pc_temp = make_PC_from_Laser(laserScan)

    # get midpoint of ranges
    # list of indexes that are in the cluster
    list_inds = []

    # init the midpoint (will be the first point)
    midPoint = pc_temp[startInd]
    # startPoint = pc_temp[startInd]

    # go through the points on the laser scan to see how big the group will get
    for cind in range(startInd + 1, stopInd):

        # the point being checked
        testPoint = pc_temp[cind]

        # finding the midpoint between the tested point and the mid point
        midPoint = ((midPoint[0] + testPoint[0]) / 2, (midPoint[1] + testPoint[1]) / 2, (midPoint[2] + testPoint[2]) / 2)
        # print("dist %f"%(dist_from_center(testPoint,midPoint)))

        # if the distance between the tested point and the middle is bigger than the radius (plus error) then the group ends
        if dist_from_center(testPoint, midPoint) > r + error:
            break
        else: #extend the group to the next point if the tested point is within the length of the radius to the midpoint
            list_inds.append(cind)
            # print("midpoint" + str(midPoint))

    # return the list of indexes in the group and the purposed midpoint of the possible circle (will be changed later with gradient descent)
    return list_inds, midPoint
    #     if indrange > max_range:
    #         continue
    #     if indrange < closest:
    #         closest = indrange
    #     midrange = midrange + indrange
    #     midrangecount += 1
    #
    # midrange = midrange / midrangecount
    #
    # midrange = (closest + midrange) / 2

    # get suspected points for a person
    # frontrange = 0.0
    # frontcount = 0
    # for cind in range(startInd, stopInd):
    #     indrange = laserScan.ranges[cind]
    #     if indrange <= midPoint and not math.isinf(indrange):
    #         list_inds.append(cind)
    #         frontrange = frontrange + laserScan.ranges[cind]
    #         frontcount += 1
    #
    #
    # if frontcount == 0:
    #     # list_inds[0] = laserScan.ranges[0]
    #     return list_inds, laserScan.ranges[startInd]
    #
    # return list_inds, frontrange / frontcount


def dist_from_center(point, center):
    square_dist = (point[0] - center[0]) ** 2 + (point[1] - center[1]) ** 2
    return square_dist ** (0.5)

# update center will find the true center of the possible circle
def update_center(points):
    global center, r

    if len(points) > 0:
        new_center = center
        gradient = (1, 1)
        while dist_from_center((0, 0), gradient) > error:
            gradient = (0, 0)
            for pt in points:
                r_prime = dist_from_center(pt, new_center)
                dldk = (r - r_prime) * ((pt[1] - new_center[1]) / r_prime)
                dldh = (r - r_prime) * ((pt[0] - new_center[0]) / r_prime)
                gradient = (gradient[0] + dldh, gradient[1] + dldk)

            gradient = (gradient[0] / len(points), gradient[1] / len(points))
            new_center = (new_center[0] - rate * gradient[0], new_center[1] - rate * gradient[1])

        print("center, new center",center,new_center)
        return new_center
    else:
        return

def get_synced_laser(image_header):
    global laserinput
    closenessInd = 0
    closeness = 10 ** 23
    teststamp = copy.deepcopy(image_header.stamp)
    imstamp = teststamp.nsecs
    imstamp += teststamp.secs * 10 ** 9

    for i in range(len(laserinput)):
        comparestamp = laserinput[i].header.stamp.nsecs
        comparestamp += laserinput[i].header.stamp.secs * 10 ** 9
        compare_closeness = math.fabs(comparestamp - imstamp)
        if compare_closeness < closeness:
            closeness = compare_closeness
            closenessInd = i

    # print(closenessInd,len(laserinput))
    synced_laser = copy.deepcopy(laserinput[closenessInd])
    synced_laser.header.frame_id = '/scan'

    laserinput = []

    return synced_laser


def ind_angle_matched(laserFrame, b_angle):
    global tfDist_cam_laser
    # print(laserSettings["angle_min"],laserSettings["angle_max"])

    angleOff_min = 100.0
    angleInd = 300
    for tfx in xrange(0, len(laserFrame.ranges)):
        ind = tfx
        laser_angle = laserSettings["angle_min"] + ind * laserSettings["angle_increment"]
        if laser_angle > math.pi / 2 or laser_angle < -math.pi / 2:
            continue
        laser_dist = laserFrame.ranges[ind]
        law_sin = (math.sin(math.pi - laser_angle) * laser_dist) / (math.sin(b_angle))
        result = laser_dist ** 2 - tfDist_cam_laser ** 2 - law_sin ** 2 + 2 * tfDist_cam_laser * law_sin * math.cos(
            b_angle)
        # print("angleoff_min",math.fabs(result))

        if math.fabs(result) < angleOff_min:
            angleInd = ind
            angleOff_min = math.fabs(result)

    # print("Angle Ind and AngleOff",angleInd,angleOff_min)
    return angleInd

# def updateHumanList(data):
#     global lAnkle, rAnkle, laserSettings, laserinput, marks, ankleMarks, adjusted_laser
#     global tf_listen, center
#     updateHumanList.lmark = Marker()
#     updateHumanList.rmark = Marker()
#     synced_scan = get_synced_laser(data.rgb_image_header)
#     # synced_scan = LaserScan()
#     # build_bg(laserinput[-1])
#     adjusted_laser.publish(synced_scan)
#
#     cloud = make_PC_from_Laser(synced_scan)
#     # projection = lp.LaserProjection()
#
#     # cloud = projection.projectLaser(synced_scan)
#     # cloud = pc.read_points(cloud,field_names=("x","y","z"))
#     # cloud = list(cloud)
#
#     testMarks = MarkerArray()
#
#     ##
#     # Working on aligning markers. Trying to find placements of known points first.
#
#     for a in range(len(data.human_list)):
#
#         leftAnkle = data.human_list[a].body_key_points_with_prob[13].x
#         rightAnkle = data.human_list[a].body_key_points_with_prob[10].x
#         leftShoulder = data.human_list[a].body_key_points_with_prob[5].x
#         rightShoulder = data.human_list[a].body_key_points_with_prob[2].x
#         chestCenter = data.human_list[a].body_key_points_with_prob[1].x
#         #
#         for oppoint in center_mass_points:
#             featur_point = data.human_list[a].body_key_points_with_prob[oppoint]
#             if featur_point.prob > 0.5:
#                 if leftShoulder < rightShoulder:
#                     if featur_point.x < leftAnkle:
#                         leftAnkle = featur_point.x
#                     if featur_point.x > rightAnkle:
#                         rightAnkle = featur_point.x
#                 else:
#                     if featur_point.x > leftAnkle:
#                         leftAnkle = featur_point.x
#                     if featur_point.x < rightAnkle:
#                         rightAnkle = featur_point.x
#
#         # pp.close()
#         # pp.clf()
#         # pp.scatter(data.human_list[a].body_key_points_with_prob[13].x,750-data.human_list[a].body_key_points_with_prob[13].y,color="g")
#         # pp.scatter(data.human_list[a].body_key_points_with_prob[10].x,750-data.human_list[a].body_key_points_with_prob[10].y,color="r")
#         # pp.scatter(data.human_list[a].body_key_points_with_prob[1].x,750-data.human_list[a].body_key_points_with_prob[1].y,color='y')
#         # pp.scatter(0,0,color="b")
#         # pp.scatter(1280,750,color="b")
#         # pp.show(block = False)
#
#         # print(laserinput)
#
#         if "angle_increment" in laserSettings and len(synced_scan.ranges) != 0:
#             # print("In true if")
#             langle = (leftAnkle - cameraOutSize[0] / 2) / camPixPerDeg
#             rangle = (rightAnkle - cameraOutSize[0] / 2) / camPixPerDeg
#             langle = langle - camAngleOff
#             rangle = rangle - camAngleOff
#             cangle = (chestCenter - cameraOutSize[0] / 2) / camPixPerDeg
#             cangle = cangle - camAngleOff
#
#             langle = math.radians(langle)
#             rangle = math.radians(rangle)
#             cangle = math.radians(cangle)
#             # print(langle/laserSettings["angle_increment"],rangle/laserSettings["angle_increment"])
#             # print(len(laserinput.ranges))
#             # lindex = int(laserSettings["array_size"] /2) - int(langle/laserSettings["angle_increment"])
#             # rindex = int(laserSettings["array_size"] /2) - int(rangle/laserSettings["angle_increment"])
#
#             lindex = ind_angle_matched(synced_scan, langle)
#             rindex = ind_angle_matched(synced_scan, rangle)
#
#             # print(lindex,rindex)
#
#             # test angle diffs
#             # angleOff = 0.0
#             # angleOff_min = 2*math.pi
#             # angleInd = 0
#             # for tfx in xrange(0,len(synced_scan.ranges)):
#             # 	ind = tfx
#             # 	laser_angle = laserSettings["angle_min"] + ind*laserSettings["angle_increment"]
#             # 	laser_dist = synced_scan.ranges[ind]
#             # 	angleOff = is_angle_matched(laser_angle,laser_dist,cangle)
#             # 	if math.fabs(angleOff) < angleOff_min:
#             # 		angleInd = ind
#             # 		angleOff_min = math.fabs(angleOff)
#
#             # print("Angle Ind and AngleOff",angleInd,angleOff_min)
#
#
#             # get mid range and subtract points farther away than midrange.
#             # midrange = 0.0;
#             # midrangecount = 0;
#             # # print(lindex,rindex)
#             # if lindex < rindex:
#             # 	for xind in range(lindex-3,rindex+4):
#             # 		if synced_scan.ranges[xind] >max_range:
#             # 			# midrange = midrange + synced_scan.ranges[xind-1]
#             # 			continue
#             # 			# print("likely Inf")
#             # 		midrange = midrange + synced_scan.ranges[xind]
#             # 		midrangecount = midrangecount +1
#             # 		# print(synced_scan.ranges[xind])
#             # else:
#             # 	for xind in range(rindex-3,lindex+4):
#             # 		if synced_scan.ranges[xind] >max_range:
#             # 			# midrange = midrange + synced_scan.ranges[xind-1]
#             # 			continue
#             # 			# print("likely Inf")
#             # 		midrange = midrange + synced_scan.ranges[xind]
#             # 		midrangecount = midrangecount +1
#             # 		# print(synced_scan.ranges[xind])
#
#
#             # midrange = midrange/midrangecount
#             # # print(midrange,midrangecount)
#
#             # frontrange = 0.0
#             # frontcount = 0
#             # if lindex<rindex:
#             # 	for xind in range(lindex-3,rindex+4):
#             # 		if synced_scan.ranges[xind] <= midrange:
#             # 			frontrange = frontrange + synced_scan.ranges[xind]
#             # 			frontcount = frontcount +1
#             # else:
#             # 	for xind in range(rindex-3,lindex+4):
#             # 		if synced_scan.ranges[xind] <= midrange:
#             # 			frontrange = frontrange + synced_scan.ranges[xind]
#             # 			frontcount = frontcount +1
#
#
#             # frontrange = frontrange/frontcount
#
#             empty, frontrange = getCluster(synced_scan, lindex, rindex)
#
#             # update frontrange
#
#
#             # print("front",frontrange,frontcount)
#
#             # center_of_mass = (-frontrange*math.tan((langle+rangle)/2),frontrange*math.sin((langle + rangle)/2))
#             # center_of_mass = (frontrange*math.sin((langle + rangle)/2),-frontrange*math.tan((langle+rangle)/2))
#             # center_of_mass = (-frontrange*math.tan((langle+rangle)/2),frontrange)
#             # center_of_mass = (frontrange,-frontrange*math.tan((langle+rangle)/2))
#             center_of_mass = (frontrange * math.cos(cangle), -frontrange * math.sin(cangle))
#             if center == (0, 0):
#                 center = (center_of_mass[0], -center_of_mass[1])
#
#             testMarks.markers.append(Marker())
#             testMarks.markers[-1].id = 0
#             testMarks.markers[-1].lifetime = rospy.Duration(show_time)
#             testMarks.markers[-1].pose = Pose(Point(center_of_mass[0], center_of_mass[1], 0), Quaternion(0, 0, 0, 1))
#             testMarks.markers[-1].type = Marker.SPHERE
#             testMarks.markers[-1].scale = Vector3(.15, .15, .15)
#             testMarks.markers[-1].action = 0
#             testMarks.markers[-1].color = ColorRGBA(0, .5, .6, 1)
#             testMarks.markers[-1].header = Header(frame_id="scan")
#             # testMarks.markers[-1].frame_locked = True
#             testMarks.markers[-1].ns = "CenterOfMass"
#             ankleMarks.publish(testMarks)
#
#             #
#             # left marker range
#             for xind in xrange(lindex - 3, lindex + 4):
#                 # print(xind)
#                 # print(len(cloud))
#                 testMarks.markers.append(Marker())
#                 testMarks.markers[-1].id = xind
#                 testMarks.markers[-1].lifetime = rospy.Duration(show_time)
#                 testMarks.markers[-1].pose = Pose(Point(cloud[xind][0], cloud[xind][1], 0), Quaternion(0, 0, 0, 1))
#                 testMarks.markers[-1].type = Marker.CUBE
#                 testMarks.markers[-1].scale = Vector3(.05, .05, .05)
#                 testMarks.markers[-1].action = 0
#                 testMarks.markers[-1].color = ColorRGBA(0, .5, 1, 1)
#                 testMarks.markers[-1].header = Header(frame_id="scan")
#                 # testMarks.markers[-1].frame_locked = True
#                 testMarks.markers[-1].ns = "left_ankle_range"
#
#             # ankleMarks.publish(testMarks)
#             # left marker range
#             for xind in xrange(rindex - 3, rindex + 4):
#                 testMarks.markers.append(Marker())
#                 testMarks.markers[-1].id = xind
#                 testMarks.markers[-1].lifetime = rospy.Duration(show_time)
#                 testMarks.markers[-1].pose = Pose(Point(cloud[xind][0], cloud[xind][1], 0), Quaternion(0, 0, 0, 1))
#                 testMarks.markers[-1].type = Marker.SPHERE
#                 testMarks.markers[-1].scale = Vector3(.05, .05, .05)
#                 testMarks.markers[-1].action = 0
#                 testMarks.markers[-1].color = ColorRGBA(0.5, 0, 1, 1)
#                 testMarks.markers[-1].header = Header(frame_id="scan")
#                 # testMarks.markers[-1].frame_locked = True
#                 testMarks.markers[-1].ns = "right_ankle_range"
#
#             # print("Sending Marks")
#             ankleMarks.publish(testMarks)
#
#             ldist = synced_scan.ranges[lindex]
#             rdist = synced_scan.ranges[rindex]
#             lAnkle = (-ldist * math.tan(langle), ldist)
#             rAnkle = (-rdist * math.tan(rangle), rdist)
#
#             # print("Laser Array Left Ankle = %d , Laser Array Right Ankle = %d"%(int(laserSettings["array_size"] /2) - int(langle/laserSettings["angle_increment"]),int(laserSettings["array_size"] /2) - int(rangle/laserSettings["angle_increment"])))
#
#             # # print("Left Ankle = (%f,%f) , Right Ankle = (%f,%f)"%(lAnkle[1],lAnkle[0],rAnkle[1],rAnkle[0]))
#             # #left Ankle
#             updateHumanList.lmark.id = 0
#             updateHumanList.lmark.lifetime = rospy.Duration(show_time)
#             updateHumanList.lmark.pose = Pose(Point(lAnkle[1], lAnkle[0], 0), Quaternion(0, 0, 0, 1))
#             updateHumanList.lmark.type = Marker.SPHERE
#             updateHumanList.lmark.scale = Vector3(.1, .1, .1)
#             updateHumanList.lmark.action = 0
#             updateHumanList.lmark.color = ColorRGBA(0, .5, 1, 1)
#             updateHumanList.lmark.header = Header(frame_id="scan")
#             updateHumanList.lmark.ns = "left_ankle"
#
#             # #right Ankle
#             updateHumanList.rmark.id = 1
#             updateHumanList.rmark.lifetime = rospy.Duration(show_time)
#             updateHumanList.rmark.pose = Pose(Point(rAnkle[1], rAnkle[0], 0), Quaternion(0, 0, 0, 1))
#             updateHumanList.rmark.type = Marker.SPHERE
#             updateHumanList.rmark.scale = Vector3(.1, .1, .1)
#             updateHumanList.rmark.action = 0
#             updateHumanList.rmark.color = ColorRGBA(.5, 0, 1, 1)
#             updateHumanList.rmark.header = Header(frame_id="scan")
#             updateHumanList.rmark.ns = "right_ankle"
#
#             marks.markers = [updateHumanList.lmark, updateHumanList.rmark]
#             ankleMarks.publish(marks)
#             # adjusted_laser.publish(synced_scan)
#             # print(marks)

def make_PC_from_Laser(laser_in):

    # point_cloud_out = PointCloud()
    # projection = lp.LaserProjection()
    #
    # cloud = projection.projectLaser(laser_in)#,channel_options = 0x04)
    # cloud = pc.read_points(cloud,field_names=("x","y","z"))#,"distances"))
    # cloud = list(cloud)
    # # print("Point from Projection")
    # # print(cloud[0])
    #
    # point_cloud_out.header = copy.deepcopy(laser_in.header)
    # point_cloud_out.channels.append(ChannelFloat32())
    # point_cloud_out.channels[0].name = "intensity"
    #
    # for a in cloud:
    #     point_cloud_out.points.append(Point(a[0],a[1],a[2]))
    #     point_cloud_out.channels[0].values.append(.99)

    updateLaser(laser_in)
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
    # return cloud


def updateLaser(data):

    global laserinput, lastLaser

    lastLaser = copy.deepcopy(data)

    laserinput.append(copy.deepcopy(data))
    # print("updateLaser")
    if "angle_increment" not in laserSettings:
        laserSettings["angle_min"] = data.angle_min
        laserSettings["angle_max"] = data.angle_max
        laserSettings["array_size"] = len(data.ranges)
        laserSettings["angle_increment"] = data.angle_increment
        laserSettings["range_min"] = data.range_min
        laserSettings["range_max"] = data.range_max
        # build_bg(data)
# def build_bg(laser_scan_in):
#     global bg_pub, adjusted_laser
#     laser_scan_out = copy.deepcopy(laser_scan_in)
#     # build_bg.map =[]
#     cloud_conv = make_PC_from_Laser(laser_scan_in)
#
#     tempPose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
#     foreground = BG.getForeground(cloud_conv, tempPose, laser_scan_in.angle_min, laser_scan_in.angle_max)
#
#     adjusted_laser.publish(laser_scan_out)
#     bg_pub.publish(foreground)
#
#     return laser_scan_out


def showAnkleMarkers():
    global ankleMarks, bg_pub, tf_listen
    global ind_list, center, center_index
    global laserList, laserSettings

    # initalize the node for ROS
    rospy.init_node("ankle_markers")

    # initilze the subscribers and the publishers for the laser, point cloud, tf, and the markers
    laserList = rospy.Subscriber("/hog/scan0", LaserScan, updateLaser)
    circleMarks = rospy.Publisher("/possible_circles", MarkerArray, queue_size=10)
    bg_pub = rospy.Publisher('/bg_cloud', PointCloud, queue_size=10)
    tf_listen = tf.TransformListener(True, rospy.Duration(1.0))

    while not rospy.is_shutdown():

        # the MarkerArray will store all the circles to be put on Rviz
        testMarks = MarkerArray()

        # if there is a laser scan being read in
        if lastLaser :
            # closest = max_range
            # midrange = 0.0
            # midrangecount = 0
            # for cind in range(center_index - 20, center_index + 20):
            # 	indrange = lastLaster.ranges[cind]
            # 	if indrange > max_range:
            # 		continue
            # 	if indrange < closest:
            # 		closest = indrange
            # 	midrange = midrange + indrange
            # 	midrangecount = midrangecount +1

            # midrange = midrange/midrangecount

            # midrange = (closest + midrange)/2

            # for cind in range(center_index - 20, center_index+20):
            # 	indrange = lastLaser.ranges[cind]
            # 	if indrange <= midrange:
            # 		ind_list.append(cind)
            # print("last laser works")

            # update the laser scan (this line may not be needed)
            updateLaser(lastLaser)

            # convert the new laser scan into a point cloud
            pcloud = make_PC_from_Laser(lastLaser)

            # possible laser point clusters that could make up a circle
            clusters = []

            # init the angle index being checked (start at 0)
            center_index = 0

            # loop through all the possible indexes
            while center_index < len(lastLaser.ranges):

                # list of indexes making up the cluster
                ind_list = []

                # finding where to start the cluster (if the point cloud is not inf)
                if not math.isinf(lastLaser.ranges[center_index]):
                    # obtaining the list of indexes making up the cluster and the possible center of the circle
                    ind_list, midrange = getCluster(lastLaser, center_index, len(lastLaser.ranges)) # search to the whole scan until the shape is gone
                    # points needed to be considered a circle threshold
                    if len(ind_list) > 50:
                        # if the shape can be considered to be a circle, then add the midpoint (key) and the index list making up the portion of the circle
                        clusters.append((midrange, ind_list))

                    # iterate to the next laser point available
                    center_index += len(ind_list) + 1
                else: #if the certain index at this angle is inf, don't bother running through the algorithm
                    # iterate to the next laser point available
                    center_index += 1

            # Updating the centers for each group to find the correct center of the circle using gradient descent
            id = 0 # seperate id used for each marker in the marker array

            # iterate through each cluster
            for g in clusters:
                print(g)

                # list of points in each cluster
                # this array will be run through update_center (gradient descent) to find the true center of the circle
                point_for_update = []

                # add each point (from point cloud) given the indexes in the cluster
                # possible error: do the indexes of the laser scan and the point cloud match?
                for i in range(g[1][0], g[1][-1]):
                    # add the x,y,z rectangular point into the array for use in update_center
                    point_for_update.append(pcloud[i])
                # print(point_for_update)

                # updated center with the given points in this cluster
                updated_center = update_center(point_for_update)

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
                testMarks.markers[-1].pose = Pose(Point(updated_center[0] + trans[0], updated_center[1] + trans[1], 0 + trans[2]), Quaternion(0, 0, 0, 1))
                testMarks.markers[-1].type = Marker.SPHERE
                testMarks.markers[-1].scale = Vector3(r, r, .01)
                testMarks.markers[-1].action = 0
                testMarks.markers[-1].color = ColorRGBA(.3, .5, .6, 1)
                testMarks.markers[-1].header = Header(frame_id="map")
                testMarks.markers[-1].ns = "circles"
                id += 1 # keep the ids unique

            # publish the marker array to be dispalyed on Rviz
            circleMarks.publish(testMarks)
            print("****************************************************************************88")

                # print("updated" + str(updated_center))

                # rate = rospy.Rate(1)
                # circleMarker = Marker()
                # circleMarker.header.frame_id = "/map"
                # circleMarker.header.stamp = rospy.get_rostime()
                # circleMarker.ns = "circles"
                # circleMarker.id = 0
                # circleMarker.type = Marker.SPHERE
                # circleMarker.action = 0
                # circleMarker.pose.position.x = updated_center[0]
                # circleMarker.pose.position.y = updated_center[1]
                # circleMarker.pose.position.z = 0
                # circleMarker.pose.orientation.x = 0
                # circleMarker.pose.orientation.y = 0
                # circleMarker.pose.orientation.z = 0.0
                # circleMarker.pose.orientation.w = 1.0
                # circleMarker.color = ColorRGBA(.3, .5, .6, .1)
                # circleMarker.lifetime = rospy.Duration(10)
                # circleMarks.publish(circleMarker)

            # rospy.spin()


            # # print(groups)
            #
            # # print(ind_list,len(lastLaser.ranges),len(pcloud))
            # personPoints = []
            # # print(len(pcloud))
            # for pt_in_cloud in ind_list:
            #     personPoints.append(pcloud[pt_in_cloud])
            # # testMarks.markers.append(Marker())
            # # testMarks.markers[-1].id = pt_in_cloud
            # # testMarks.markers[-1].lifetime = rospy.Duration(show_time)
            # # testMarks.markers[-1].type = Marker.CUBE
            # # testMarks.markers[-1].scale = Vector3(.05,.05,.05)
            # # testMarks.markers[-1].action = 0
            # # testMarks.markers[-1].color = ColorRGBA(.5,.5,1,1)
            # # testMarks.markers[-1].header = Header(frame_id = "base")
            # # # testMarks.markers[-1].frame_locked = True
            # # testMarks.markers[-1].ns = "circle_points"
            # # print(personPoints)
            #
            # center = update_center(personPoints)
            # print(center)
            # # center_index = ind_angle(math.atan2(center[1], center[0]))
            # print("center index %d"%(center_index))
            # ind_list = []
            # testMarks.markers.append(Marker())
            # testMarks.markers[-1].id = 0
            # testMarks.markers[-1].lifetime = rospy.Duration(show_time)
            # testMarks.markers[-1].pose = Pose(Point(center[0], center[1], -.2), Quaternion(0, 0, 0, 1))
            # testMarks.markers[-1].type = Marker.SPHERE
            # testMarks.markers[-1].scale = Vector3(r, r, .01)
            # testMarks.markers[-1].action = 0
            # testMarks.markers[-1].color = ColorRGBA(.3, .5, .6, 1)
            # testMarks.markers[-1].header = Header(frame_id="map")
            # # testMarks.markers[-1].frame_locked = True
            # testMarks.markers[-1].ns = "circles"
            # circleMarks.publish(testMarks)

            # rospy.spin()


if __name__ == '__main__':
    showAnkleMarkers()