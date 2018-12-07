#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import json
import math
from RangeSensor import RangeSensor

range_sensors_frame_ids = ["ultrasonic_front_emission_point", "ultrasonic_back_emission_point", "ultrasonic_left", "ultrasonic_right"]
update_rate = 15

def main():
	range_sensors = []

	#TODO: change minimum rango an appropraite distance
	print ("[DEBUG] range_sensors: change minimum range an appropraite distance!")

	for frame_id in range_sensors_frame_ids:
		range_sensors.append(
			RangeSensor(
				frame_id=frame_id,
				min_range = 0.0,
				max_range = 0.352,
				min_angle = math.radians(-15.0),
				max_angle = math.radians(15.0)
				)
			)

	range_publisher = rospy.Publisher("/scan", LaserScan, queue_size=10)
	rospy.init_node("range_sensors_ros_handler_node", anonymous=False)

	def callback(data):
		data = json.loads(data.data)

		# rospy.loginfo("I heard {data}!".format(data=data))

		for index in range(min(len(data), len(range_sensors))):
			range_sensors[index].update_range(data[index])
			scan_msg = range_sensors[index].get_as_laserscan()
			range_publisher.publish(scan_msg)

	rospy.Subscriber("raw_range_data", String, callback)

	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException as e:
		# raise e
		pass