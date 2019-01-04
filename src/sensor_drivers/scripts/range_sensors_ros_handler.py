#!/usr/bin/env python

import math
import json
import rospy
import tf2_ros
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import TransformStamped
from laser_scanner import RangeSensor, LaserScannerUsingRangeSensors

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
    scan_publisher = rospy.Publisher("/scan", LaserScan, queue_size=10)
    range_publisher = rospy.Publisher("/ultrasonic_data", Range, queue_size=10)
    
    rospy.init_node("range_sensors_ros_handler_node", anonymous=False)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    my_laser_scanner = LaserScannerUsingRangeSensors(parent_origin_frame_id, "laser_scan_center_point", tf_buffer)
    my_laser_scanner.set_number_of_active_sensors(4)

    range_sensors = []
    for frame_id in range_sensors_frame_ids:
        range_sensors.append(
                RangeSensor(parent_origin_frame_id, frame_id, tf_buffer)
            )
    
    rospy.loginfo("[INFO] loading and calculating transforms for the sensor objects")
    while True:
        if my_laser_scanner.update_transform():
            break
        else:
            rospy.sleep(0.2)
    
    for range_sensor in range_sensors:
        while True:
            if range_sensor.update_transform():
                break
            else:
                rospy.sleep(0.2)
    rospy.loginfo("[INFO] finished loading and calculating transforms for the sensor objects")

    # manually reseting rotations, TODO: fix later
    range_sensors[0].relative_transform.set_rotation(0)              # front
    range_sensors[1].relative_transform.set_rotation(math.pi / 2.0)  # right
    range_sensors[2].relative_transform.set_rotation(math.pi)        # back
    range_sensors[3].relative_transform.set_rotation(-math.pi / 2.0) # left

    # my_laser_scanner.relative_transform.set_rotation(math.pi)
    # my_laser_scanner.compute()

    def callback(data):
        measured_distances = json.loads(data.data)
        publish_ultrasonic_messages(range_publisher, measured_distances)

        for index in range(min(len(measured_distances), len(range_sensors))):
            range_sensors[index].update(measured_distances[index])

    rospy.Subscriber("raw_range_data", String, callback)
    # rospy.spin()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        my_laser_scanner.update(range_sensors)
        my_laser_scanner.publish_message(scan_publisher)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException as e:
        # raise e
        pass