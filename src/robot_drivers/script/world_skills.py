#!/usr/bin/env python
from __future__ import division
from decimal import *
import thread
import rospy, signal
from std_msgs.msg import String
from robot_interface_advanced import AdvancedRobotInterface

rospy.init_node("eurobot_task_handler", anonymous=False)
rospy.loginfo("initializing the robot...")
robot = AdvancedRobotInterface()
robot.initialize()

#TODO
# trojkat prostokatny z bokiem x (drive to wall)
# d = robot.move_to_surface(target_speed=speed_level[0], move_timeout=8)
# robot.move_linear(-d, target_speed=speed_level[0], move_timeout=8)


def drive_square(dist):
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.12)
    robot.move_angular(90)
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.12)
    robot.move_angular(90)
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.12)
    robot.move_angular(90)
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.12)
    robot.move_angular(90)


def drive_triangle_isosceles(dist):
    robot.move_angular(30)
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.12)
    robot.move_angular(120)
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.12)
    robot.move_angular(120)
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.12)
    robot.move_angular(120)


# def drive_triangle_equilateral(dist):
# TODO

while True:
    x = str(raw_input('Please choose(Square:s, Isosceles Triangle:ts, Equilateral Triangle:te): '))
    print("You've chosen option %s" % x)

    y = int(input("Please give a side length"))
    print("Your side lenght is: %i" % y)

    if x == 's' or x == 'S':
        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        drive_square(y)
    elif x == 'ts' or x == 'Ts':
        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        drive_triangle_isosceles(y)
    elif x == 'te' or x == 'TE':
        rospy.loginfo("waiting for pull to start")
        robot.wait_for_pull_to_start(state=True)
        rospy.sleep(0.1)
        drive_triangle_equilateral(y)

rospy.loginfo("ctrl-c to terminate")
rospy.spin()

rospy.loginfo("terminating....")
robot.terminate()

