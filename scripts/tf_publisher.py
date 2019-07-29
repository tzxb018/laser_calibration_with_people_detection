#!/usr/bin/env python
import rospy
import tf
from lc.srv import Laser_tf, Detection_target
from sensor_msgs.msg import LaserScan, PointCloud2, PointCloud, ChannelFloat32
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
import copy
import message_filters
import numpy
from lc.msg import matrix_tf



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
# def callback_hog_matrx(data):
#     global hog_matrix, hog_ref_point_c, hog_ref_point_a
#     basis = numpy.array([[data.matrix_tf[0], data.matrix_tf[2]], [data.matrix_tf[1], data.matrix_tf[3]]])
#     hog_matrix = basis
#     hog_ref_point_c = (data.matrix_tf[4], data.matrix_tf[5])
#     hog_ref_point_a = (data.matrix_tf[6], data.matrix_tf[7])
#
# def callback_hog_pc(data):
#     global hog_pc, hog_pc_header
#
#     hog_pc_header = data.header
#     hog_pc = data.points
#
# def callback_mouse_matrix(data):
#     global mouse_matrix, mouse_ref_point_c, mouse_ref_point_a
#     basis = numpy.array([[data.matrix_tf[0], data.matrix_tf[2]], [data.matrix_tf[1], data.matrix_tf[3]]])
#     mouse_matrix = basis
#     mouse_ref_point_c = (data.matrix_tf[4], data.matrix_tf[5])
#     mouse_ref_point_a = (data.matrix_tf[6], data.matrix_tf[7])
#
# def callback_mouse_pc(data):
#     global mouse_pc, mouse_pc_header
#
#     mouse_pc_header = data.header
#     mouse_pc = data.points

def callback_hog_laser(data):
    return
    global lastLaser_hog, laser_pub_hog, time_hog
    time_hog = data.header.stamp.nsecs
    print("hog  : " + str(data.header.stamp.nsecs))
    # laser_pub_hog.publish(data)

    # lastLaser_hog = copy.deepcopy(data)
    #
    # # print("callback laser hog")
    #
    # if "angle_increment" not in laserSettings_hog:
    #     laserSettings_hog["angle_min"] = data.angle_min
    #     laserSettings_hog["angle_max"] = data.angle_max
    #     laserSettings_hog["array_size"] = len(data.ranges)
    #     laserSettings_hog["angle_increment"] = data.angle_increment
    #     laserSettings_hog["range_min"] = data.range_min
    #     laserSettings_hog["range_max"] = data.range_max

    # lastLaser_hog.header.stamp = rospy.Time.now()
    # laser_pub_hog.publish(lastLaser_hog)

def callback_mouse_laser(data):
    return
    global lastLaser_mouse, laser_pub_mouse, time_mouse

    time_mouse = data.header.stamp.nsecs
    print("mouse: " + str(data.header.stamp.nsecs))
    # laser_pub_mouse.publish(data)

    # lastLaser_mouse = copy.deepcopy(data)
    # if "angle_increment" not in laserSettings_mouse:
    #     laserSettings_mouse["angle_min"] = data.angle_min
    #     laserSettings_mouse["angle_max"] = data.angle_max
    #     laserSettings_mouse["array_size"] = len(data.ranges)
    #     laserSettings_mouse["angle_increment"] = data.angle_increment
    #     laserSettings_mouse["range_min"] = data.range_min
    #     laserSettings_mouse["range_max"] = data.range_max

    # lastLaser_mouse.header.stamp = rospy.Time.now()
    # laser_pub_mouse.publish(lastLaser_mouse)

def callback_snake_laser(data):
    return
    global lastLaser_snake, laser_pub_snake, time_snake

    time_snake = data.header.stamp.nsecs
    print("snake: " + str(data.header.stamp.nsecs))
    # laser_pub_snake.publish(data)
    # lastLaser_snake = copy.deepcopy(data)
    # if "angle_increment" not in laserSettings_snake:
    #     laserSettings_snake["angle_min"] = data.angle_min
    #     laserSettings_snake["angle_max"] = data.angle_max
    #     laserSettings_snake["array_size"] = len(data.ranges)
    #     laserSettings_snake["angle_increment"] = data.angle_increment
    #     laserSettings_snake["range_min"] = data.range_min
    #     laserSettings_snake["range_max"] = data.range_max

    # lastLaser_snake.header.stamp = rospy.Time.now()
    # laser_pub_snake.publish(lastLaser_snake)

def callback_duck_laser(data):
    return
    global lastLaser_duck, laser_pub_duck, time_duck

    time_duck = data.header.stamp.nsecs
    #
    # lastLaser_duck = copy.deepcopy(data)
    # if "angle_increment" not in laserSettings_duck:
    #     laserSettings_duck["angle_min"] = data.angle_min
    #     laserSettings_duck["angle_max"] = data.angle_max
    #     laserSettings_duck["array_size"] = len(data.ranges)
    #     laserSettings_duck["angle_increment"] = data.angle_increment
    #     laserSettings_duck["range_min"] = data.range_min
    #     laserSettings_duck["range_max"] = data.range_max
    #
    # lastLaser_duck.header.stamp = rospy.Time.now()
    # laser_pub_duck.publish(lastLaser_duck)
def callback_all(hog, mouse, snake):
    return
    global lastLaser_duck, lastLaser_mouse, lastLaser_hog, lastLaser_snake, laser_pub

    print("sync*****")
    print("running lasers hog   " + str(hog.header.stamp.nsecs))
    print("running lasers snake " + str(snake.header.stamp.nsecs))
    print("running lasers mouse " + str(mouse.header.stamp.nsecs))
    print("sync*****")


    laser_pub_hog.publish(hog)
    laser_pub_mouse.publish(mouse)
    laser_pub_snake.publish(snake)
    # laser_pub.publish(duck)
    # print(hog)

    # lastLaser_hog = copy.deepcopy(hog)
    # lastLaser_mouse = copy.deepcopy(mouse)
    # lastLaser_snake = copy.deepcopy(snake)
    # lastLaser_duck = copy.deepcopy(duck)
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
    global lastLaser_hog, laser_sub_hog, laser_pub_hog
    global lastLaser_mouse, laser_sub_mouse, laser_pub_mouse
    global lastLaser_snake, laser_sub_snake, laser_pub_snake
    global lastLaser_duck, laser_sub_duck, laser_pub_duck, laser_pub
    global time_hog, time_mouse, time_duck, time_snake


    rospy.init_node('laser_tf_broadcaster')
    rate = rospy.Rate(1)

    laser_sub_hog = message_filters.Subscriber("/hog/scan0", LaserScan)
    laser_sub_mouse = message_filters.Subscriber("/mouse/scan0", LaserScan)
    laser_sub_snake = message_filters.Subscriber("/snake/scan0", LaserScan)

    # laser_sub_hog = rospy.Subscriber("/hog/scan0", LaserScan, callback_hog_laser)
    # laser_sub_mouse = rospy.Subscriber("/mouse/scan0", LaserScan, callback_mouse_laser)
    # laser_sub_snake = rospy.Subscriber("/snake/scan0", LaserScan, callback_snake_laser)



    # laser_sub_h = rospy.Subscriber("/hog/scan0", LaserScan, callback_hog_laser)
    # laser_sub_m = rospy.Subscriber("/mouse/scan0", LaserScan, callback_mouse_laser)
    # laser_sub_s = rospy.Subscriber("/snake/scan0", LaserScan, callback_snake_laser)

    # laser_sub_duck = message_filters.Subscriber("/duck/scan0", LaserScan)
    # laser_pub = rospy.Publisher("/updatedScan", LaserScan, queue_size=1)
    #
    #
    laser_pub_hog = rospy.Publisher("/hog/updatedScan", LaserScan, queue_size=1)
    laser_pub_mouse = rospy.Publisher("/mouse/updatedScan", LaserScan, queue_size=1)
    laser_pub_snake = rospy.Publisher("/snake/updatedScan", LaserScan, queue_size=1)
    # laser_pub_duck = rospy.Publisher("/duck/updatedScan", LaserScan, queue_size=10)
    #
    #
    # # hog_sub = rospy.Subscriber("/matrix_for_hog", matrix_tf, callback_hog_matrx)
    # # hog_bg_sub = rospy.Subscriber('/hog/bg_cloud', PointCloud, callback_hog_pc)
    # #
    # # mouse_sub = rospy.Subscriber("/matrix_for_mouse", matrix_tf, callback_mouse_matrix)
    # # mouse_bg_sub = rospy.Subscriber('/mouse/bg_cloud', PointCloud, callback_mouse_pc)
    #
    # # callback_hog_matrx(rospy.wait_for_message("/matrix_for_hog", matrix_tf))
    # # callback_hog_pc(rospy.wait_for_message('/hog/bg_cloud', PointCloud))
    # # callback_mouse_matrix(rospy.wait_for_message("/matrix_for_mouse", matrix_tf))
    # # callback_mouse_pc(rospy.wait_for_message('/mouse/bg_cloud', PointCloud))
    #
    # callback_hog_laser(rospy.wait_for_message('/hog/scan0', LaserScan))
    # callback_mouse_laser(rospy.wait_for_message('/mouse/scan0', LaserScan))
    # callback_snake_laser(rospy.wait_for_message('/snake/scan0', LaserScan))
    # callback_duck_laser(rospy.wait_for_message('/duck/scan0', LaserScan))

    laser_target_finder = rospy.ServiceProxy('detection_target', Detection_target)



    # calls the service for finding the calibration target
    # print(lastLaser_hog)
    reu = laser_target_finder("/hog/scan0")
    hog_ref_point_a = reu.point_a
    hog_ref_point_c = reu.point_c

    rev = laser_target_finder("/mouse/scan0")
    mouse_ref_point_a = rev.point_a
    mouse_ref_point_c = rev.point_c

    rew = laser_target_finder("/snake/scan0")
    snake_ref_point_a = rew.point_a
    snake_ref_point_c = rew.point_c

    # rex = laser_target_finder("/duck/scan0")
    # duck_ref_point_a = rex.point_a
    # duck_ref_point_c = rex.point_c

    # print("hog a\n" + str(hog_ref_point_a))
    # print("hog c\n" + str(hog_ref_point_c))
    # print("mouse a\n" + str(mouse_ref_point_a))
    # print("mouse c\n" + str(mouse_ref_point_c))
    # print("snake a\n" + str(snake_ref_point_a))
    # print("snake c\n" + str(snake_ref_point_c))
    # print("duck a\n" + str(duck_ref_point_a))
    # print("duck c\n" + str(duck_ref_point_c))

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


    res = laser_tf_listener(Point(snake_ref_point_c.x, snake_ref_point_c.y, 0),
                            Point(hog_ref_point_c.x, hog_ref_point_c.y, 0),
                            Point(snake_ref_point_a.x, snake_ref_point_a.y, 0),
                            Point(hog_ref_point_a.x, hog_ref_point_a.y, 0))
    br1 = tf.TransformBroadcaster()
    print("tf2\n" + str(res))

    # rer = laser_tf_listener(Point(duck_ref_point_c.x, duck_ref_point_c.y, 0),
    #                         Point(hog_ref_point_c.x, hog_ref_point_c.y, 0),
    #                         Point(duck_ref_point_a.x, duck_ref_point_a.y, 0),
    #                         Point(hog_ref_point_a.x, hog_ref_point_a.y, 0))
    # br2 = tf.TransformBroadcaster()
    # print("tf3\n" + str(rer))

    ts = message_filters.ApproximateTimeSynchronizer([laser_sub_hog, laser_sub_mouse, laser_sub_snake],
                                                     1, 1)

    # find highest time of all the lasers, have the rest of lasers catch up to start sync

    # # call the laser update to publish the updated laser scan
    # callback_hog_laser(rospy.wait_for_message('/hog/scan0', LaserScan))
    # callback_mouse_laser(rospy.wait_for_message('/mouse/scan0', LaserScan))
    # callback_snake_laser(rospy.wait_for_message('/snake/scan0', LaserScan))
    # callback_duck_laser(rospy.wait_for_message('/duck/scan0', LaserScan))
    #
    # print("TIMES")
    # print(time_hog)
    # print(time_mouse)
    # print(time_snake)
    #
    # max_time = max(time_hog, time_mouse, time_snake)
    # while time_hog < max_time:
    #     if time_hog < max_time:
    #         callback_hog_laser(rospy.wait_for_message('/hog/scan0', LaserScan))
    # while time_mouse < max_time:
    #     if time_mouse < max_time:
    #         callback_mouse_laser(rospy.wait_for_message('/mouse/scan0', LaserScan))
    # while time_snake < max_time:
    #     if time_snake < max_time:
    #         callback_snake_laser(rospy.wait_for_message('/snake/scan0', LaserScan))
    #
    # print("init sync finished:")
    # print(max_time)
    # print(time_hog)
    # print(time_mouse)
    # print(time_snake)

    while not rospy.is_shutdown():

        # hog is the reference laser for all the lasers to orient to
        # first tf (mouse to hog)
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

        # second tf (snake to hog)
        br.sendTransform((res.source_translation.x, res.source_translation.y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, res.source_theta),
                         rospy.Time.now(),
                         "laser_hog_target",
                         "laser_hog")

        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, res.theta),
                         rospy.Time.now(),
                         "laser_snake_target",
                         "laser_hog_target")

        br.sendTransform((res.dest_translation.x, res.dest_translation.y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, res.dest_theta),
                         rospy.Time.now(),
                         "laser_snake",
                         "laser_snake_target")



        ts.registerCallback(callback_all)

        # third tf (duck to hog)
        # br.sendTransform((rer.source_translation.x, rer.source_translation.y, 0),
        #                  tf.transformations.quaternion_from_euler(0, 0, rer.source_theta),
        #                  rospy.Time.now(),
        #                  "laser_hog_target",
        #                  "laser_hog")
        #
        # br.sendTransform((0, 0, 0),
        #                  tf.transformations.quaternion_from_euler(0, 0, rer.theta),
        #                  rospy.Time.now(),
        #                  "laser_duck_target",
        #                  "laser_hog_target")
        #
        # br.sendTransform((rer.dest_translation.x, rer.dest_translation.y, 0),
        #                  tf.transformations.quaternion_from_euler(0, 0, rer.dest_theta),
        #                  rospy.Time.now(),
        #                  "laser_duck",
        #                  "laser_duck_target")

        # # call the laser update to publish the updated laser scan
        # callback_hog_laser(rospy.wait_for_message('/hog/scan0', LaserScan))
        # callback_mouse_laser(rospy.wait_for_message('/mouse/scan0', LaserScan))
        # callback_snake_laser(rospy.wait_for_message('/snake/scan0', LaserScan))
        # callback_duck_laser(rospy.wait_for_message('/duck/scan0', LaserScan))

        rate.sleep()

if __name__ == '__main__':
    publish_tf()









