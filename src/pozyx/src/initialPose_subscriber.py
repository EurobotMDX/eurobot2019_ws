#!/usr/bin/env python
import rospy
import time as t
from geometry_msgs.msg import PoseWithCovarianceStamped

def initial_pos_pub():
    publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.init_node('initialPose_subscriber.py', anonymous=True)
    #Creating the message with the type PoseWithCovarianceStamped
    rospy.loginfo("This node sets the x position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")
    start_pos = PoseWithCovarianceStamped()
    #filling header with relevant information
    start_pos.header.frame_id = "map"
    start_pos.header.stamp = rospy.Time.now()
    #filling payload with relevant information gathered from subscribing
    # to initialpose topic published by RVIZ via rostopic echo initialpose
    start_pos.pose.pose.position.x = -0.846684932709
    start_pos.pose.pose.position.y = 0.333061099052
    start_pos.pose.pose.position.z = 0.0

    start_pos.pose.pose.orientation.x = 0.0
    start_pos.pose.pose.orientation.y = 0.0
    start_pos.pose.pose.orientation.z = -0.694837665627
    start_pos.pose.pose.orientation.w = 0.719166613815

    start_pos.pose.covariance[0] = 0.25
    start_pos.pose.covariance[7] = 0.25
    start_pos.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    start_pos.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    start_pos.pose.covariance[35] = 0.06853891945200942

    rospy.loginfo(start_pos)
    publisher.publish(start_pos)

if __name__ == '__main__':
    t.sleep(5)
    try:
        initial_pos_pub()
    except rospy.ROSInterruptException:
        pass
