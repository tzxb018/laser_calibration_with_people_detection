#!/usr/bin/env python
import rospy
import tf
from lc.srv import Laser_tf
import math
from geometry_msgs.msg import Point


def laser_tf(req):

    source_point_c = req.source_point_c
    dest_point_c = req.dest_point_c
    source_point_a = req.source_point_a
    dest_point_a = req.dest_point_a

    translation = (float(dest_point_c.x - source_point_c.x), float(dest_point_c.y - source_point_c.y))

    # change between reference point (right angle) and short leg in the frame it is being published

    delta_source = ((source_point_a.x - source_point_c.x), (source_point_a.y - source_point_c.y))

    # finding the angle between hog's short leg and mouse's short leg to find relative orientation
    delta_dest = ((dest_point_a.x - dest_point_c.x), (dest_point_a.y - dest_point_c.y))

    theta_dest = math.atan2(delta_dest[1], delta_dest[0])

    theta_source = math.atan2(delta_source[1], delta_source[0])

    theta = theta_dest - theta_source

    # returns the message for laser_tf
    # float32 source_theta, geometry_msgs/Point source_translation, float32 theta, float32 dest_theta, geometry_msgs/Point dest_translation
    return 0, Point(dest_point_c.x, dest_point_c.y, 0), theta, 0, Point(-source_point_c.x, -source_point_c.y, 0)

def laser_tf_server():
    rospy.init_node('laser_Tf')
    s = rospy.Service('laser_tf', Laser_tf, laser_tf)
    rospy.spin()

if __name__ == '__main__':
    laser_tf_server()



