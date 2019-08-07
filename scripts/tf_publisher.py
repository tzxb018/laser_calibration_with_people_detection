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
import sensor_msgs.point_cloud2 as pc
import laser_geometry as lp
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA

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

pc_li_hog = PointCloud()
pc_li_snake = PointCloud()
pc_li_mouse = PointCloud()
pc_display = PointCloud()

cyan = ColorRGBA(.3, 1, 1, 1)

def callback_hog_laser(data):
    global lastLaser_hog, laser_pub_hog, time_hog, pc_li_hog
    time_hog = data.header.stamp
    # print("hog  : " + str(data.header.stamp.nsecs))
    pc_li_hog = make_PC_from_Laser_display(data)
    lastLaser_hog = data


def callback_mouse_laser(data):
    global lastLaser_mouse, laser_pub_mouse, time_mouse, pc_li_mouse

    time_mouse = data.header.stamp
    # print("mouse: " + str(data.header.stamp.nsecs))
    pc_li_mouse = make_PC_from_Laser_display(data)


def callback_snake_laser(data):
    global lastLaser_snake, laser_pub_snake, time_snake, pc_li_snake

    time_snake = data.header.stamp
    # print("snake: " + str(data.header.stamp.nsecs))
    pc_li_snake = make_PC_from_Laser_display(data)


def callback_duck_laser(data):
    global lastLaser_duck, laser_pub_duck, time_duck

    time_duck = data.header.stamp.nsecs

def callback_all(hog, mouse, snake):
    global lastLaser_duck, lastLaser_mouse, lastLaser_hog, lastLaser_snake, laser_pub
    global pc_li_hog, pc_li_mouse, pc_li_snake, pc_display

    # print("sync*****")
    # print("running lasers hog   " + str(hog.header.stamp.nsecs))
    # print("running lasers snake " + str(snake.header.stamp.nsecs))
    # print("running lasers mouse " + str(mouse.header.stamp.nsecs))
    # print("sync*****")

    # pc_li_hog = make_PC_from_Laser_display(hog)
    # pc_li_mouse = make_PC_from_Laser_display(mouse)
    # pc_li_snake = make_PC_from_Laser_display(snake)
    #
    # for pt in pc_li_hog.points:
    #     pc_display.points.append(pt)
    #
    # for pt in pc_li_snake.points:
    #     pc_display.points.append(pt)
    #
    # for pt in pc_li_mouse.points:
    #     pc_display.points.append(pt)

    #
    # laser_pub_hog.publish(hog)
    # laser_pub_mouse.publish(mouse)
    # laser_pub_snake.publish(snake)

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


def publish_tf():
    global lastLaser_hog, laser_sub_hog, laser_pub_hog
    global lastLaser_mouse, laser_sub_mouse, laser_pub_mouse
    global lastLaser_snake, laser_sub_snake, laser_pub_snake
    global lastLaser_duck, laser_sub_duck, laser_pub_duck, laser_pub
    global time_hog, time_mouse, time_duck, time_snake
    global pc_display, pc_li_mouse, pc_li_snake, pc_li_hog


    rospy.init_node('laser_tf_broadcaster')
    rate = rospy.Rate(40)

    laser_sub_hog = rospy.Subscriber("/hog/scan0", LaserScan, callback_hog_laser)
    laser_sub_mouse = rospy.Subscriber("/mouse/scan0", LaserScan, callback_mouse_laser)
    laser_sub_snake = rospy.Subscriber("/snake/scan0", LaserScan, callback_snake_laser)
    # laser_sub_duck = rospy.Subscriber("/duck/scan0", LaserScan, callback_duck_laser)

    laser_target_finder = rospy.ServiceProxy('detection_target', Detection_target)

    triangle_marks = rospy.Publisher("/possible/triangles", MarkerArray, queue_size=10)
    testMarks = MarkerArray()
    id = 0

    # calls the service for finding the calibration target
    # print(lastLaser_hog)
    reu = laser_target_finder("/hog/scan0")
    hog_ref_point_a = reu.point_a
    hog_ref_point_c = reu.point_c
    hog_ref_point_b = reu.point_b

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


    pc_display.header.frame_id = "/map"
    while not rospy.is_shutdown():

        # grab emssage time from emssage
        common_time = time_hog

        # hog is the reference laser for all the lasers to orient to
        # first tf (mouse to hog)
        br.sendTransform((ret.source_translation.x, ret.source_translation.y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, ret.source_theta),
                         common_time,
                         "laser_hog_target",
                         "laser_hog")

        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, ret.theta),
                         common_time,
                         "laser_mouse_target",
                         "laser_hog_target")

        br.sendTransform((ret.dest_translation.x, ret.dest_translation.y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, ret.dest_theta),
                         common_time,
                         "laser_mouse",
                         "laser_mouse_target")

        # second tf (snake to hog)
        # br.sendTransform((res.source_translation.x, res.source_translation.y, 0),
        #                  tf.transformations.quaternion_from_euler(0, 0, res.source_theta),
        #                  common_time,
                         # "laser_hog_target",
                         # "laser_hog")

        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, res.theta),
                         common_time,
                         "laser_snake_target",
                         "laser_hog_target")

        br.sendTransform((res.dest_translation.x, res.dest_translation.y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, res.dest_theta),
                         common_time,
                         "laser_snake",
                         "laser_snake_target")

        callback_hog_laser(rospy.wait_for_message('/hog/scan0', LaserScan))
        callback_mouse_laser(rospy.wait_for_message('/mouse/scan0', LaserScan))
        callback_snake_laser(rospy.wait_for_message('/snake/scan0', LaserScan))
        # callback_duck_laser(rospy.wait_for_message('/duck/scan0', LaserScan))

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

        # add a new marker to the marker array
        testMarks.markers.append(Marker())

        # keep the marker ids unique
        testMarks.markers[-1].id = id

        # determining how long the markers will stay up in Rviz
        testMarks.markers[-1].lifetime = rospy.Duration(0.0)
        testMarks.markers[-1].type = Marker.LINE_STRIP
        testMarks.markers[-1].scale = Vector3(.03, .03, .01)
        testMarks.markers[-1].action = 0
        testMarks.markers[-1].color = cyan
        testMarks.markers[-1].points = [hog_ref_point_a, hog_ref_point_c]
        testMarks.markers[-1].header = lastLaser_hog.header  # Header(frame_id="/map")
        testMarks.markers[-1].ns = "final_triangle"
        id += 1  # keep the ids unique

        # add a new marker to the marker array
        testMarks.markers.append(Marker())

        # keep the marker ids unique
        testMarks.markers[-1].id = id

        # determining how long the markers will stay up in Rviz
        testMarks.markers[-1].lifetime = rospy.Duration(0.0)
        testMarks.markers[-1].type = Marker.LINE_STRIP
        testMarks.markers[-1].scale = Vector3(.03, .03, .01)
        testMarks.markers[-1].action = 0
        testMarks.markers[-1].color = cyan
        testMarks.markers[-1].points = [hog_ref_point_b, hog_ref_point_c]
        testMarks.markers[-1].header = lastLaser_hog.header  # Header(frame_id="/map")
        testMarks.markers[-1].ns = "final_triangle"
        id += 1  # keep the ids unique

        # add a new marker to the marker array
        testMarks.markers.append(Marker())

        # keep the marker ids unique
        testMarks.markers[-1].id = id

        # determining how long the markers will stay up in Rviz
        testMarks.markers[-1].lifetime = rospy.Duration(0.0)
        testMarks.markers[-1].type = Marker.LINE_STRIP
        testMarks.markers[-1].scale = Vector3(.03, .03, .01)
        testMarks.markers[-1].action = 0
        testMarks.markers[-1].color = cyan
        testMarks.markers[-1].points = [hog_ref_point_a, hog_ref_point_b]
        testMarks.markers[-1].header = lastLaser_hog.header  # Header(frame_id="/map")
        testMarks.markers[-1].ns = "final_triangle"
        id += 1  # keep the ids unique

        # Publish triangle
        triangle_marks.publish(testMarks)
        
        rate.sleep()

if __name__ == '__main__':
    publish_tf()









