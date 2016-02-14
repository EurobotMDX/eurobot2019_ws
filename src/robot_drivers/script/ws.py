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



def do_generic(robot, speed_level=[0.2, 0.25, 0.38, 0.4]):
    global SHOULD_RUN

    rospy.loginfo("closing grippers....")
    # must be closed first right than left
    robot.close_gripper_right()
    time.sleep(.2)

    robot.close_gripper_left()
    time.sleep(0.5)

    rospy.loginfo("waiting for pull to start")
    # state=True
    robot.wait_for_pull_to_start(state=False)
    rospy.sleep(2)

    # do something before each run, before pull to start
    # something

    rospy.loginfo("waiting for pull to start")
    robot.wait_for_pull_to_start(state=True)

#     do something after pull to start before each run
#     robot.move_angular(360)



def test_robot(robot):
    global SHOULD_RUN

    robot.open_gripper_left()
    time.sleep(2)

    robot.open_gripper_right()
    time.sleep(2)

    robot.close_gripper_right()
    time.sleep(2)

    robot.close_gripper_left()
    time.sleep(2)



    # robot.move_angular(360)
    # robot.move_linear(0.1, should_avoid_obstacles=True, obstacle_backup_distance=0.2, clearing_distance=0.32)

def main():
    do_generic(robot)
    test_robot(robot)

    robot.proximity_sensor_left_callback(range_msg)
    robot.proximity_sensor_front_callback(range_msg)
    robot.proximity_sensor_right_callback(range_msg)


if __name__== "__main__":
    main()

rospy.loginfo("ctrl-c to terminate")
rospy.spin()

rospy.loginfo("terminating....")
robot.terminate()
