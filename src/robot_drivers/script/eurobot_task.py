#!/usr/bin/env python
from __future__ import division

import rospy, signal
from robot_interface_advanced import AdvancedRobotInterface







rospy.init_node("eurobot_task_handler", anonymous=False)
rospy.loginfo("initializing the robot...")

robot = AdvancedRobotInterface()
robot.initialize()

rospy.loginfo("waiting for pull to start")
robot.wait_for_pull_to_start()

# robot.move_linear(1.2)
# robot.move_angular(90)
# robot.move_angular(90)
# robot.move_linear(1.2)

robot.move_to(1.0, 0.0, 90, 0.4)

rospy.loginfo("ctrl-c to terminate")
rospy.spin()

rospy.loginfo("terminating....")
robot.terminate()
