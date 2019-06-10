#!/usr/bin/env python
from __future__ import division

import thread
import rospy, signal
from std_msgs.msg import String
from robot_interface_advanced import AdvancedRobotInterface


rospy.init_node("eurobot_task_handler", anonymous=False)

rospy.loginfo("initializing the robot...")

robot = AdvancedRobotInterface()
robot.initialize()

rospy.loginfo("waiting for pull to start")
robot.wait_for_pull_to_start(state=True); rospy.sleep(3.0)

# Code goes here

robot.move_linear(0.5, should_avoid_obstacles=True )
robot.move_angular(90)
robot.move_linear(0.1, should_avoid_obstacles=True)
robot.move_angular(90)
robot.move_linear(0.4, should_avoid_obstacles=True)


rospy.loginfo("ctrl-c to terminate")
rospy.spin()

rospy.loginfo("terminating....")
robot.terminate()

