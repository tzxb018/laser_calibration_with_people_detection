#!/usr/bin/env python
import rospy
import tf
from lc.srv import Laser_tf
import math
from geometry_msgs.msg import Point


def laser_tf(req):

    # dest = hog, source = mouse
    source_point_c = req.source_point_c
    dest_point_c = req.dest_point_c
    source_point_a = req.source_point_a
    dest_point_a = req.dest_point_a

    translation = (float(dest_point_c.x - source_point_c.x), float(dest_point_c.y - source_point_c.y))

    # mouse_to_hog = (float(hog_ref_point_c[0] - mouse_ref_point_c[0]), float(hog_ref_point_c[1] - mouse_ref_point_c[1]))

    # adding all the points to the hog reference frame

    # change between reference point (right angle) and short leg in the frame it is being published
    # delta hog = delta x , delta y
    # delta_hog = ((hog_ref_point_a[0] - hog_ref_point_c[0]), (hog_ref_point_a[1] - hog_ref_point_c[1]))
    delta_source = ((source_point_a.x - source_point_c.x), (source_point_a.y - source_point_c.y))

    # delta_mouse = ((mouse_ref_point_a[0] - mouse_ref_point_c[0]), (mouse_ref_point_a[1] - mouse_ref_point_c[1]))

    # hog_to_mouse_a = ((mouse_ref_point_a[0] + delta_hog[0]), (mouse_ref_point_a[1] + delta_hog[1]))

    # finding the angle between hog's short leg and mouse's short leg to find relative orientation
    # delta_q = ((mouse_ref_point_a[0] - mouse_ref_point_c[0]), (mouse_ref_point_a[1] - mouse_ref_point_c[1]))
    # delta_q = ((hog_ref_point_a[0] - hog_ref_point_c[0]), (hog_ref_point_a[1] - hog_ref_point_c[1]))
    delta_dest = ((dest_point_a.x - dest_point_c.x), (dest_point_a.y - dest_point_c.y))


    # theta_q = math.atan2(delta_q[1], delta_q[0])
    theta_dest = math.atan2(delta_dest[1], delta_dest[0])

    # theta_hog = math.atan2(delta_hog[1], delta_hog[0])
    # theta_mouse = math.atan2(delta_mouse[1], delta_mouse[0])
    theta_source = math.atan2(delta_source[1], delta_source[0])


    # theta = theta_q - theta_hog
    # theta = theta_q - theta_mouse
    theta = theta_dest - theta_source
    return (0, Point(dest_point_c.x, dest_point_c.y, 0), theta, 0, Point(-source_point_c.x, -source_point_c.y, 0))

    # return (theta_dest, Point(dest_point_c.x, dest_point_c.y, 0), -theta, theta_source, Point(-source_point_c.x, -source_point_c.y, 0))
#         source_theta
# Point source_translation
# float32 theta
# float32 dest_theta
# Point dest_translation



def laser_tf_server():
    rospy.init_node('laser_Tf')
    s = rospy.Service('laser_tf', Laser_tf, laser_tf)
    rospy.spin()

if __name__ == '__main__':
    laser_tf_server()



