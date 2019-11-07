#!/usr/bin/env python
from __future__ import division
from decimal import *
import time
import thread
import rospy, signal
from std_msgs.msg import String
from robot_interface_advanced import AdvancedRobotInterface

rospy.init_node("eurobot_task_handler", anonymous=False)
rospy.loginfo("initializing the robot...")
robot = AdvancedRobotInterface()
robot.initialize()

rospy.loginfo("waiting for pull to start")
robot.wait_for_pull_to_start(state=True)


rospy.loginfo("Testing Grippers")
robot.open_gripper()
time.sleep(1)
robot.close_gripper()
time.sleep(1)


rospy.loginfo("Testing Half Grippers")
robot.semi_open_gripper()
time.sleep(1)
robot.semi_close_gripper()
time.sleep(1)


rospy.loginfo("Testing Angular Velocity")
robot.move_angular(180)
robot.move_angular(180)
robot.move_angular(180)
robot.move_angular(180)


rospy.loginfo("Testing Linear Velocity")
# robot.move_linear(0.1, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)


rospy.loginfo("ctrl-c to terminate")
rospy.spin()

rospy.loginfo("terminating....")
robot.terminate()
