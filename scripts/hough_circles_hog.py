#!/usr/bin/env python

import cv2
import math
import numpy as np
import matplotlib.pyplot as plt
import rospy
import copy
from sensor_msgs.msg import LaserScan

laserSettings, laserinput = ({}, [])

def updateLaser(data):
    # Initialize global variables for LaserScan data
    global laserSettings, pointList

    magnitude = data.range_max * 10

    # Initialize the image to a white blank RGB image
    blank_image = np.ones((magnitude * 2, magnitude * 2, 3), np.uint8) * 255

    for i in range(0, len(data.ranges)):
        # Getting angle in radians (each index represents .004/6.28)
        theta = data.angle_increment * i

        # Polar to rectangular conversion (x = r cos(), y = r sin())
        xcoor = theta * data.ranges[i] * math.cos(theta)
        ycoor = theta * data.ranges[i] * math.sin(theta)

        # If not inf, then plot the point in black
        if abs(xcoor) < magnitude or abs(ycoor) < magnitude:
            blank_image[int(10 * round(xcoor,1) + magnitude), int( 10 * round(ycoor,1) + magnitude)] = [0,0,0]
            blank_image[int(10 * round(xcoor, 1) + magnitude) + 1, int(10 * round(ycoor, 1) + magnitude)] = [0, 0, 0]
            blank_image[int(10 * round(xcoor, 1) + magnitude), int(10 * round(ycoor, 1) + magnitude) + 1] = [0, 0, 0]
            blank_image[int(10 * round(xcoor, 1) + magnitude)+ 1, int(10 * round(ycoor, 1) + magnitude) + 1] = [0, 0, 0]

def hough():
    img = cv2.imread('/home/tbessho/catkin_ws/src/laser_camera_calibration/scripts/coins.jpg', 0)
    img = cv2.medianBlur(img, 5)
    cimg = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 50,
                               param1=30, param2=80, minRadius=0, maxRadius=0)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # draw the outer circle
            cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # draw the center of the circle
            cv2.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)

    cv2.imshow('detected circles', cimg)
    cv2.waitKey(0)
    cv2.destroyAllWindows()




        # if "angle_increment" not in laserSettings:
        #     laserSettings["angle_min"] = data.angle_min
        #     laserSettings["angle_max"] = data.angle_max
        #     laserSettings["array_size"] = len(data.ranges)
        #     laserSettings["angle_increment"] = data.angle_increment
        #     laserSettings["range_min"] = data.range_min
        #     laserSettings["range_max"] = data.range_max
        # build_bg(data)


if __name__ == '__main__':


    # Naming the new node that can be used in launch files
    # rospy.init_node('hough_hog')

    # Create the subscriber for the LaserScan
    # laserList = rospy.Subscriber("/hog/scan0", LaserScan, updateLaser, queue_size=1)

    hough()

    # rospy.spin()
