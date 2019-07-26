#!/usr/bin/env python
import rospy
import tf
from lc.srv import Laser_tf, Detection_target
from sensor_msgs.msg import LaserScan, PointCloud2, PointCloud, ChannelFloat32
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
import copy
import message_filters
import numpy
import sys

laser_hog, laser_mouse, laser_snake = [], [], []

hog_matrix = numpy.array([[]])
hog_pc = []
hog_pc_header = []
hog_ref_point_c = (0, 0)
hog_ref_point_a = (0, 0)
laserSettings_hog = {}
lastLaser_hog, laser_pub_hog, laser_sub_hog = LaserScan(), [], []

mouse_matrix = numpy.array([[]])
mouse_pc = []
mouse_pc_header = []
mouse_ref_point_c = (0,0)
mouse_ref_point_a = (0,0)
laserSettings_mouse = {}
lastLaser_mouse, laser_pub_mouse, laser_sub_mouse = LaserScan(), [], []

snake_matrix = numpy.array([[]])
snake_pc = []
snake_pc_header = []
snake_ref_point_c = (0,0)
snake_ref_point_a = (0,0)
laserSettings_snake = {}
lastLaser_snake, laser_pub_snake, laser_sub_snake = LaserScan(), [], []

duck_matrix = numpy.array([[]])
duck_pc = []
duck_pc_header = []
duck_ref_point_c = (0,0)
duck_ref_point_a = (0,0)
laserSettings_duck = {}
lastLaser_duck, laser_pub_duck, laser_sub_duck = LaserScan(), [], []

laser_pub = []
time_hog = 0
time_mouse = 0
time_snake = 0
time_duck = 0

margin = float(sys.argv[1])

laser_pub_hog, laser_pub_mouse, laser_pub_snake = [], [], []

def callback_hog_laser(data):
    global lastLaser_hog, laser_pub_hog, time_hog
    time_hog = float(str(data.header.stamp.secs) + "." + str(data.header.stamp.nsecs))
    print("hog  : " + str(time_hog))
    # laser_pub_hog.publish(data)

def callback_mouse_laser(data):
    global lastLaser_mouse, laser_pub_mouse, time_mouse
    time_mouse =  float(str(data.header.stamp.secs) + "." + str(data.header.stamp.nsecs))
    print("mouse: " + str(time_mouse))
    # laser_pub_mouse.publish(data)

def callback_snake_laser(data):
    global lastLaser_snake, laser_pub_snake, time_snake
    time_snake =  float(str(data.header.stamp.secs) + "." + str(data.header.stamp.nsecs))
    print("snake: " + str(time_snake))
    # laser_pub_snake.publish(data)

def callback_hog_laser_publish(data):
    global lastLaser_hog, laser_pub_hog, time_hog
    time_hog = float(str(data.header.stamp.secs) + "." + str(data.header.stamp.nsecs))
    print("hog publish  : " + str(time_hog))
    laser_pub_hog.publish(data)

def callback_mouse_laser_publish(data):
    global lastLaser_mouse, laser_pub_mouse, time_mouse
    time_mouse =  float(str(data.header.stamp.secs) + "." + str(data.header.stamp.nsecs))
    print("mouse publish: " + str(time_mouse))
    laser_pub_mouse.publish(data)

def callback_snake_laser_publish(data):
    global lastLaser_snake, laser_pub_snake, time_snake
    time_snake =  float(str(data.header.stamp.secs) + "." + str(data.header.stamp.nsecs))
    print("snake publish: " + str(time_snake))
    laser_pub_snake.publish(data)

# def callback_duck_laser(data):
#     global lastLaser_duck, laser_pub_duck, time_duck
#     time_duck = data.header.stamp.secs

def callback_all(hog, mouse, snake):
    global laser_pub_hog, laser_pub_mouse, laser_pub_snake

    print("sync*****")
    print("running lasers hog   " + str(hog.header.stamp.secs))
    print("running lasers snake " + str(snake.header.stamp.secs))
    print("running lasers mouse " + str(mouse.header.stamp.secs))
    print("sync*****")

    laser_pub_hog.publish(hog)
    laser_pub_mouse.publish(mouse)
    laser_pub_snake.publish(snake)


def sync_time():
    global laser_pub_hog, laser_pub_mouse, laser_pub_snake, time_mouse, time_snake, time_hog, margin

    rospy.init_node('time_syncronizer')

    laser_sub_hog = message_filters.Subscriber("/hog/scan0", LaserScan)
    laser_sub_mouse = message_filters.Subscriber("/mouse/scan0", LaserScan)
    laser_sub_snake = message_filters.Subscriber("/snake/scan0", LaserScan)

    laser_pub_hog = rospy.Publisher("/hog/updatedScan", LaserScan, queue_size=1)
    laser_pub_mouse = rospy.Publisher("/mouse/updatedScan", LaserScan, queue_size=1)
    laser_pub_snake = rospy.Publisher("/snake/updatedScan", LaserScan, queue_size=1)

    # ts = message_filters.ApproximateTimeSynchronizer([laser_sub_hog, laser_sub_mouse, laser_sub_snake],
    #                                                  1, 1)
    # find highest time of all the lasers, have the rest of lasers catch up to start sync

    # call the laser update to publish the updated laser scan
    callback_hog_laser(rospy.wait_for_message('/hog/scan0', LaserScan))
    callback_mouse_laser(rospy.wait_for_message('/mouse/scan0', LaserScan))
    callback_snake_laser(rospy.wait_for_message('/snake/scan0', LaserScan))
    # callback_duck_laser(rospy.wait_for_message('/duck/scan0', LaserScan))

    print("TIMES")
    print(time_hog)
    print(time_mouse)
    print(time_snake)
    max_time = max(time_hog, time_snake, time_mouse)
    print("MAX  : " + str(max_time))
    while time_hog < max_time:
        if time_hog < max_time:
            callback_hog_laser(rospy.wait_for_message('/hog/scan0', LaserScan))
    while time_mouse < max_time:
        if time_mouse < max_time:
            callback_mouse_laser(rospy.wait_for_message('/mouse/scan0', LaserScan))
    while time_snake < max_time:
        if time_snake < max_time:
            callback_snake_laser(rospy.wait_for_message('/snake/scan0', LaserScan))

    print("init sync finished:")
    print("MAX  : " + str(max_time))
    print(time_hog)
    print(time_mouse)
    print(time_snake)
    frames = 0
    while not rospy.is_shutdown():
        # ts.registerCallback(callback_all)

        hog, snake, mouse = False, False, False

        print("MAX  : " + str(max_time))


        while max_time - margin < time_hog < max_time + margin:
            callback_hog_laser(rospy.wait_for_message('/hog/scan0', LaserScan))
            hog = True
        while max_time - margin < time_mouse < max_time + margin:
            callback_mouse_laser(rospy.wait_for_message('/mouse/scan0', LaserScan))
            mouse = True
        while max_time - margin < time_snake < max_time + margin:
            callback_snake_laser(rospy.wait_for_message('/snake/scan0', LaserScan))
            snake = True

        # if hog == False:
        #     callback_hog_laser(rospy.wait_for_message('/hog/scan0', LaserScan))
        #     hog = True
        #
        # if mouse == False:
        #     callback_mouse_laser(rospy.wait_for_message('/mouse/scan0', LaserScan))
        #     mouse = True
        #
        # if snake == False:
        #     callback_snake_laser(rospy.wait_for_message('/snake/scan0', LaserScan))
        #     snake = True

        frames += 1
        max_time = max(time_hog, time_snake, time_mouse)
        # need to callback the max time's laser
        callback_hog_laser_publish(rospy.wait_for_message('/hog/scan0', LaserScan))
        callback_mouse_laser_publish(rospy.wait_for_message('/mouse/scan0', LaserScan))
        callback_snake_laser_publish(rospy.wait_for_message('/snake/scan0', LaserScan))

        print("total frames: " + str(frames))
        print("****************************************************************")
        # callback_duck_laser(rospy.wait_for_message('/duck/scan0', LaserScan))


if __name__ == '__main__':
    sync_time()