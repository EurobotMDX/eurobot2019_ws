#!/usr/bin/env python

from __future__ import division

import rospy
import signal
from robot_interface_advanced import AdvancedRobotInterface

def move_to_position(robot, x, y, yaw=0):
    rospy.loginfo("[INFO] moving to x:{}, y:{}, yaw:{}".format(x, y, yaw))
    robot.move_to(x, y, yaw)
    
    rospy.loginfo("done")
    rospy.sleep(2)
    

rospy.init_node("eurobot_task_handler", anonymous=False)
robot = AdvancedRobotInterface()

rospy.loginfo("initializing the robot")
robot.initialize()

should_run = True

try:
    rospy.loginfo("waiting for pull to start")
    robot.wait_for_pull_to_start()
except KeyboardInterrupt:
    should_run = False
    rospy.loginfo("setting should_run to False")

if should_run:
    move_to_position(robot, 0, 1.3909, 90.0)
    robot.move_linear(-0.63948)
    move_to_position(robot, 0.0, 0.12975, 0.0)

rospy.loginfo("ctrl-c to terminate")
rospy.spin()

rospy.loginfo("terminating....")
robot.terminate()
