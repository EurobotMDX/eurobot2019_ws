#!/usr/bin/env python
from __future__ import division
from decimal import *

import time
import thread
import rospy, signal
from std_msgs.msg import String
from robot_interface_advanced import AdvancedRobotInterface

# rospy.init_node("eurobot_task_handler", anonymous=False)
# rospy.loginfo("initializing the robot...")
# robot = AdvancedRobotInterface()
# robot.initialize()

SHOULD_RUN = False
THREAD_LOCKED = False

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
    robot.move_angular(360)


def line(robot, speed_level=[0.2, 0.25, 0.38, 0.4]):
    global SHOULD_RUN

    rospy.loginfo("starting task line following")

    do_generic(robot)

    if not SHOULD_RUN:
        robot.stop_motors()
        return

    robot.move_linear(0.1, target_speed=speed_level[2])

    rospy.loginfo("done with task line following")

def obstacle(robot, speed_level=[0.2, 0.25, 0.38, 0.4]):
    global SHOULD_RUN


    rospy.loginfo("starting task obstacle avoidance")

    do_generic(robot)

    if not SHOULD_RUN:
        robot.stop_motors()
    return

    robot.move_linear(0.1, target_speed=speed_level[2])

    rospy.loginfo("done with task obstacle avoidance")

def sorting(robot, speed_level=[0.2, 0.25, 0.38, 0.4]):
    global SHOULD_RUN


    rospy.loginfo("starting task pucks sorting")

    do_generic(robot)

    if not SHOULD_RUN:
        robot.stop_motors()
    return

    robot.move_linear(0.1, target_speed=speed_level[2])

    rospy.loginfo("done with task pucks sorting")

# def eurobot_task_cmd_callback(msg):
#     global THREAD_LOCKED
#
#     if not THREAD_LOCKED:
#         thread.start_new_thread(eurobot_task_cmd_handler, (msg,))
#
# rospy.Subscriber('eurobot_task_cmd', String, eurobot_task_cmd_callback)



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

rospy.init_node("world_skills_task_handler", anonymous=False)
rospy.loginfo("initializing the robot...")

robot = AdvancedRobotInterface()
robot.initialize()

def world_skills_task_cmd_handler(msg):
    global robot, SHOULD_RUN, THREAD_LOCKED

    command = msg.data
    rospy.loginfo("[INFO] world_skills_task_cmd_handler task received the command: {}".format(command))

    if command == "reset":
        rospy.loginfo("[INFO] world_skills_task_cmd_handler task received command to reset")

        if SHOULD_RUN:
            SHOULD_RUN = False
            rospy.sleep(10)

        rospy.loginfo("terminating robot interface....")
        robot.terminate()

        rospy.loginfo("initializing robot interface....")
        robot.initialize()

        rospy.loginfo("resetting robot odometry....")
        robot.reset_odometry()

        rospy.loginfo("opening grippers....")
        robot.open_gripper()

        rospy.loginfo("pushing left....")
        robot.push_left()

        rospy.sleep(0.5)

        rospy.loginfo("closing grippers....")
        robot.close_gripper()

        rospy.loginfo("pushing right....")
        robot.push_right()

        robot.robot_score = 0.0

    elif command == "start_line":
        rospy.loginfo("[INFO] world_skills_task_cmd_handler task received command to start line following")

        if not SHOULD_RUN:
            SHOULD_RUN = True
            thread.start_new_thread(table_1_yellow, (robot,))
            rospy.loginfo("[INFO] started line following task")
        else:
            rospy.loginfo("world_skills_task_cmd_handler task is already running")

    elif command == "start_obstacle":
        rospy.loginfo("[INFO] world_skills_task_cmd_handler task received command to start avoid obstacle")

        if not SHOULD_RUN:
            SHOULD_RUN = True
            thread.start_new_thread(table_1_purple, (robot,))
            rospy.loginfo("[INFO] started obstacle avoidance task")
        else:
            rospy.loginfo("world_skills_task_cmd_handler task is already running")

    elif command == "start_sorting":
        rospy.loginfo("[INFO] world_skills_task_cmd_handler task received command to start sorting")

        if not SHOULD_RUN:
            SHOULD_RUN = True
            thread.start_new_thread(table_1_purple, (robot,))
            rospy.loginfo("[INFO] started obstacle avoidance task")
        else:
            rospy.loginfo("world_skills_task_cmd_handler task is already running")

    elif command == "test_line":
        rospy.loginfo("[INFO] world_skills_task_cmd_handler task received command to test robot line following")

        thread.start_new_thread(test_robot, (robot,))

    elif command == "test_obstacle":
        rospy.loginfo("[INFO] world_skills_task_cmd_handler task received command to test robot sorting pucks")

        thread.start_new_thread(test_robot, (robot,))

    elif command == "test_sorting":
        rospy.loginfo("[INFO] world_skills_task_cmd_handler task received command to test robot sorting pucks")

        thread.start_new_thread(test_robot, (robot,))

    elif command == "test":
        rospy.loginfo("[INFO] eurobot task received command to test robot")

        thread.start_new_thread(test_robot, (robot,))

    elif command == "kill_task":
        SHOULD_RUN = False

def world_skills_task_cmd_callback(msg):
    global THREAD_LOCKED

    if not THREAD_LOCKED:
        thread.start_new_thread(world_skills_task_cmd_handler, (msg,))

rospy.Subscriber('world_skills_task_cmd', String, world_skills_task_cmd_callback)

rospy.loginfo("ctrl-c to terminate")
rospy.spin()

rospy.loginfo("terminating....")
robot.terminate()
