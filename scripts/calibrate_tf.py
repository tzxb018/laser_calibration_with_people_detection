#!/usr/bin/env python  
import rospy
import tf2_ros
import geometry_msgs.msg
import math
import tf

if __name__ == '__main__':
    rospy.init_node('calibrate_hog')
    t = geometry_msgs.msg.TransformStamped()
    br = tf2_ros.TransformBroadcaster()
    t.header.frame_id = "map"
    t.child_frame_id = "laser_hog"

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
	time = rospy.Time.now().to_sec()
        t.header.stamp = rospy.Time.now()

	for i in range(0,3600,1):	
		quaternion = tf.transformations.quaternion_from_euler(0, 0, 0.01*i)
	        t.transform.translation.x = 2.0
        	t.transform.translation.y = 0.0
      		t.transform.translation.z = 0.0
       		t.transform.rotation.x = quaternion[0]
       		t.transform.rotation.y = quaternion[1]
       		t.transform.rotation.z = quaternion[2]
       		t.transform.rotation.w = quaternion[3]
	        br.sendTransform(t)
       		rate.sleep()
