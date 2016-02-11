#!/usr/bin/env python
from __future__ import division

import thread
import rospy, signal
from std_msgs.msg import String
from robot_interface_advanced import AdvancedRobotInterface

# Simon Klimek 22 May WorldSkills task
# draw square and triangle with given length

rospy.init_node("eurobot_task_handler", anonymous=False)

rospy.loginfo("initializing the robot...")

robot = AdvancedRobotInterface()
robot.initialize()

rospy.loginfo("waiting for pull to start")
robot.wait_for_pull_to_start(state=True); rospy.sleep(3.0)

# Code goes here

# driving
def drive_square(dist):
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.12)
    robot.move_angular(90)
    robot.update_robot_score(50)
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.12)
    robot.move_angular(90)
    robot.update_robot_score(50)
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.12)
    robot.move_angular(90)
    robot.update_robot_score(50)
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.12)
    robot.move_angular(90)
    robot.update_robot_score(50)

robot.deactivate_experiment()
drive_square(0.5)

def drive_triangle():
    pass

rospy.loginfo("ctrl-c to terminate")
rospy.spin()

rospy.loginfo("terminating....")
robot.terminate()

