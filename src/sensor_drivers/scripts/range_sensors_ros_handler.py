#!/usr/bin/env python

import math
import json
import rospy
import tf2_ros
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Range, PointCloud
from geometry_msgs.msg import TransformStamped
from laser_scanner import RangeSensor

update_rate = 15
parent_origin_frame_id = "base_link"
range_sensors_frame_ids = ["ultrasonic_front_emission_point", "ultrasonic_right_emission_point", "ultrasonic_back_emission_point", "ultrasonic_left_emission_point"]

def publish_ultrasonic_messages(data_publisher, measured_distances):
    range_msg = Range()
    range_msg.radiation_type = 0 # ULTRASOUND
    range_msg.field_of_view = math.pi / 6.0 # 30 degrees
    range_msg.min_range = 0.01
    range_msg.max_range = 2.5

    for index, measured_distance in enumerate(measured_distances):
        range_msg.range = measured_distance
        range_msg.header.frame_id = range_sensors_frame_ids[index]
        range_msg.header.stamp = rospy.Time.now()

        data_publisher.publish(range_msg)

def main():
    point_cloud_publisher = rospy.Publisher("/point_cloud", PointCloud, queue_size=10)
    range_publisher = rospy.Publisher("/ultrasonic_data", Range, queue_size=10)
    
    rospy.init_node("range_sensors_ros_handler_node", anonymous=False)

    front_sensor_msg = PointCloud()
    front_sensor_msg.header.frame_id = "ultrasonic_front_emission_point"

    def callback(data):
        measured_distances = json.loads(data.data)
        publish_ultrasonic_messages(range_publisher, measured_distances)

    rospy.Subscriber("raw_range_data", String, callback)
    # rospy.spin()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        # front_sensor_msg
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException as e:
        # raise e
        pass