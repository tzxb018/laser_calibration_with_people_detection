#!/usr/bin/env python  
# import rospy
# import sensor_msgs.point_cloud2 as pc2
# import rospy
# from sensor_msgs.msg import LaserScan, PointCloud2, PointCloud, ChannelFloat32
# import laser_geometry.laser_geometry as lg
# import laser_geometry as lp
# import copy
# import math
import tf2_ros
# import tf
# import sensor_msgs.point_cloud2 as pc
import geometry_msgs.msg
import rospy
import math
import copy
# from rtree import index
import matplotlib.pyplot as pp
import tf
import laser_geometry as lp

from sensor_msgs.msg import LaserScan, PointCloud2, PointCloud, ChannelFloat32
import sensor_msgs.point_cloud2 as pc
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
# import bgsubtract as BG

# from openpose_ros_msgs.msg import *
pc_li = PointCloud()

laserSettings, laserinput =({},[])
# taken from wiki.ros.org/laser_geometry
def scan_cb(msg):
    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)

    # now we can do something with the PointCloud2 for example:
    # publish it
    pc_pub.publish(pc2_msg)

    # convert it to a generator of the individual points
    point_generator = pc2.read_points(pc2_msg)

    # we can access a generator in a loop
    sum = 0.0
    num = 0
    for point in point_generator:
        if not math.isnan(point[2]):
            sum += point[2]
            num += 1
    # we can calculate the average z value for example
    print(str(sum / num))

    # or a list of the individual points which is less efficient
    point_list = pc2.read_points_list(pc2_msg)

    # we can access the point list with an index, each element is a namedtuple
    # we can access the elements by name, the generator does not yield namedtuples!
    # if we convert it to a list and back this possibility is lost
    print(point_list[len(point_list) / 2].x)

def make_PC_from_Laser(laser_in):

    # if len(laser_list_in) < 1:
    #     return []


    # laser_in = laser_list_in
    point_cloud_out = PointCloud()
    projection = lp.LaserProjection()

    cloud = projection.projectLaser(laser_in)  # ,channel_options = 0x04)
    # print(cloud.fields)
    cloud = pc.read_points(cloud, field_names=("x", "y", "z"))  # ,"distances"))
    cloud = list(cloud)
    # print("Point from Projection")
    # print(cloud[0])

    point_cloud_out.header = copy.deepcopy(laser_in.header)
    point_cloud_out.channels.append(ChannelFloat32())
    point_cloud_out.channels[0].name = "intensity"

    for a in cloud:
        point_cloud_out.points.append(Point(a[0], a[1], a[2]))
        point_cloud_out.channels[0].values.append(.99)

    # laser_list_in.pop()
    return point_cloud_out

def updateLaser(data):
    global laserSettings, laserinput, pc_li

    laserinput.append(copy.deepcopy(data))
    # print(laserinput.ranges)
    if "angle_increment" not in laserSettings:
        laserSettings["angle_min"] = data.angle_min
        laserSettings["angle_max"] = data.angle_max
        laserSettings["array_size"] = len(data.ranges)
        laserSettings["angle_increment"] = data.angle_increment
        laserSettings["range_min"] = data.range_min
        laserSettings["range_max"] = data.range_max
    # build_bg(data)
    pc_li =  make_PC_from_Laser(data)


if __name__ == '__main__':

    # naming the new node that can be used in launch files
    rospy.init_node('calibrate_hog')

    # t = geometry_msgs.msg.TransformStamped()
    # br = tf2_ros.TransformBroadcaster()

    laserList = rospy.Subscriber("/hog/scan0", LaserScan, updateLaser, queue_size=1)
    bg_pub = rospy.Publisher('/bg_cloud', PointCloud, queue_size=10)

    # setting the transform frame names (can be found declared in the launch file)
    # t.header.frame_id = "map"
    # t.child_frame_id = "bg_cloud"

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        time = rospy.Time.now().to_sec()
        # t.header.stamp = rospy.Time.now()

        # declaring the node for publishing the new point cloud data (for comparing)
        # li_temp = make_PC_from_Laser(laserinput)
        # if pc_li != []:
            # tempPose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
            # foreground = BG.getForeground(li_temp, tempPose, laserSettings["angle_min"], laserSettings["angle_max"])
        pc_li.header.frame_id = "bg_cloud"
        bg_pub.publish(pc_li)
        # print(len(laserinput))?

        # # rotates the laser's tf in a circle
        # for i in range(0, 3600, 1):
        #     quaternion = tf.transformations.quaternion_from_euler(0, 0, 0.01 * i)
        #     t.transform.translation.x = 2.0
        #     t.transform.translation.y = 0.0
        #     t.transform.translation.z = 0.0
        #     t.transform.rotation.x = quaternion[0]
        #     t.transform.rotation.y = quaternion[1]
        #     t.transform.rotation.z = quaternion[2]
        #     t.transform.rotation.w = quaternion[3]
        #     br.sendTransform(t)
        #     rate.sleep()
