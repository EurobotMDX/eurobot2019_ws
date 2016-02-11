#!/usr/bin/env python
from __future__ import division

import thread
import rospy, signal
from std_msgs.msg import String
from robot_interface_advanced import AdvancedRobotInterface

# Simon Klimek 22 May WorldSkills task
# draw square and triangle with given length

rospy.loginfo("initializing the robot...")

robot = AdvancedRobotInterface()
robot.initialize()

rospy.loginfo("waiting for pull to start")
robot.wait_for_pull_to_start(state=True); rospy.sleep(3.0)

# Code goes here

# driving
def drive_square(dist):
    robot.move_linear(dist)
    robot.move_angular(90)
    robot.move_linear(dist)
    robot.move_angular(90)
    robot.move_linear(dist)
    robot.move_angular(90)

drive_square(0.5)

def drive_triangle():
    pass

rospy.loginfo("ctrl-c to terminate")
rospy.spin()

rospy.loginfo("terminating....")
robot.terminate()

