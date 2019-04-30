#!/usr/bin/env python
from __future__ import division

import rospy, signal
from robot_interface_advanced import AdvancedRobotInterface

def table_1_purple(robot):
    rospy.loginfo("adding score for experiment")
    robot.update_robot_score(40.0)

    robot.move_to(0.16943, 0.34233, -1.23446, target_speed=0.3)
    robot.move_to(0.05432, 1.48142, -90.18769, target_speed=0.32, should_avoid_obstacles=True)

    robot.push_left()
    d = robot.move_to_surface(target_speed=0.15, access_key="back", move_timeout=8); rospy.sleep(0.5)

    robot.push_right(); rospy.sleep(0.5)
    robot.move_linear(d, should_avoid_obstacles=True, move_timeout=8) # add should_avoid_obstacles

    rospy.loginfo("adding score for particle accelerator")
    robot.update_robot_score(10.0)

    robot.move_to(0.02273, 2.05537, 88.13082, target_speed=0.3, should_avoid_obstacles=True)
    robot.open_gripper(); rospy.sleep(0.5)

    rospy.loginfo("adding score for flap open")
    robot.update_robot_score(10.0)

    d = robot.move_to_surface(target_speed=0.15, move_timeout=8) # , move_timeout=8
    robot.close_gripper(); rospy.sleep(1.0)
    robot.move_linear(-d, target_speed=0.15, move_timeout=8) # , move_timeout=8

    rospy.loginfo("adding score for releasing goldium")
    robot.update_robot_score(20.0)

    robot.move_to(-0.8771, 1.1, -83.07387, target_speed=0.31, should_avoid_obstacles=True)
    d = robot.move_to_surface(target_speed=0.15)
    robot.open_gripper(); rospy.sleep(1.0)

    rospy.loginfo("adding score for goldium in weighing area")
    robot.update_robot_score(24.0)

def table_1_yellow(robot):
    rospy.loginfo("adding score for experiment")
    robot.update_robot_score(40.0)

    robot.move_to(-0.18233, 0.30964, -0.33545, target_speed=0.3)
    robot.move_to(-0.0151, 1.48142, 88.16924, target_speed=0.32, should_avoid_obstacles=True)

    robot.push_right()
    d = robot.move_to_surface(target_speed=0.15, access_key="back", move_timeout=8)

    robot.push_left(); rospy.sleep(0.5)
    robot.move_linear(d)

    rospy.loginfo("adding score for particle accelerator")
    robot.update_robot_score(10.0)

    robot.move_to(-0.16383, 2.0498, -87.90616, target_speed=0.31, should_avoid_obstacles=True)
    robot.open_gripper(); rospy.sleep(0.5)

    rospy.loginfo("adding score for flap open")
    robot.update_robot_score(10.0)

    d = robot.move_to_surface(target_speed=0.15, move_timeout=8)
    robot.close_gripper(); rospy.sleep(0.8)
    robot.move_linear(-d, target_speed=0.15, should_avoid_obstacles=True, move_timeout=8)

    rospy.loginfo("adding score for releasing goldium")
    robot.update_robot_score(20.0)

    robot.move_to(0.41827, 1.04506, 77.68496, target_speed=0.31, should_avoid_obstacles=True)
    robot.move_to(0.87744, 1.14051, 77.43708, target_speed=0.31)
    d = robot.move_to_surface(target_speed=0.15)
    robot.open_gripper(); rospy.sleep(1.0)

    rospy.loginfo("adding score for goldium in weighing area")
    robot.update_robot_score(24.0)


rospy.init_node("eurobot_task_handler", anonymous=False)
rospy.loginfo("initializing the robot...")

robot = AdvancedRobotInterface()
robot.initialize()

rospy.loginfo("waiting for pull to start")
robot.wait_for_pull_to_start(state=False); rospy.sleep(3.0)

robot.move_linear(0.1)

rospy.loginfo("waiting for pull to start")
robot.wait_for_pull_to_start(state=True)

#table_1_yellow(robot)
table_1_purple(robot)

rospy.loginfo("ctrl-c to terminate")
rospy.spin()

rospy.loginfo("terminating....")
robot.terminate()
