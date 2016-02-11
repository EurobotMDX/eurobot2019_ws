#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import LaserScan

class RangeSensor(object):
	def __init__(self, frame_id="", min_range=0.0, max_range=0.5, min_angle=math.radians(-90.0), max_angle=math.radians(-60.0)):
		self.frame_id = frame_id

		self.min_range = min_range
		self.max_range = max_range

		self.min_angle = min_angle
		self.max_angle = max_angle
		
		self.num_readings = 10
		self.angle_increment = abs(max_angle - min_angle) / self.num_readings

		self.laser_frequency = 1000.0

		self.ranges = []
		self.intensities = []
		self.measured_range = max_range

		# pre-populate the LaserScan message
		self.scan = LaserScan()
		self.scan.header.frame_id = self.frame_id
		self.scan.angle_min = self.min_angle
		self.scan.angle_max = self.max_angle
		self.scan.angle_increment = self.angle_increment
		self.scan.time_increment = 1.0 / (self.laser_frequency * float(self.num_readings))
		self.scan.range_min = self.min_range
		self.scan.range_max = self.max_range

	def update_range(self, measured_range=0.4):
		self.measured_range = self._constrain(measured_range, self.min_range, self.max_range)

	def get_as_laserscan(self):
		self.ranges = []
		self.intensities = []

		for i in range(self.num_readings):
			angle = (i * self.angle_increment) + self.min_angle
			d = self.measured_range / math.cos(angle)
			self.ranges.append(d)
			self.intensities.append(d)

		scan_time = rospy.Time.now()
		
		# populate the LaserScan message
		self.scan.header.stamp = scan_time
		self.scan.ranges = self.ranges
		self.scan.intensities = self.intensities

		return self.scan

	def _constrain(self, val, min_val, max_val):
		return min(max_val, max(val, min_val))