#!/usr/bin/env python


import tf
import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('world_tf_broadcaster')
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        br = tf2_ros.TransformBroadcaster()
        
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "world"

        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0

        q = tf.transformations.quaternion_from_euler(0,0,0)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        br.sendTransform(t)

        rate.sleep()