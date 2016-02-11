#!/usr/bin/env python
from __future__ import division

import thread
import rospy, signal
from std_msgs.msg import String
from robot_interface_advanced import AdvancedRobotInterface

SHOULD_RUN = False

def do_generic(robot):
    global SHOULD_RUN

    rospy.loginfo("closing grippers....")
    robot.close_gripper()

    rospy.sleep(0.5)

    rospy.loginfo("opening grippers....")
    robot.open_gripper()

    rospy.loginfo("waiting for pull to start")
    robot.wait_for_pull_to_start(state=False); rospy.sleep(3.0)

    robot.move_linear(0.1)
    
    rospy.loginfo("waiting for pull to start")
    robot.wait_for_pull_to_start(state=True)

    rospy.loginfo("adding score for experiment")
    robot.update_robot_score(40.0)

def table_1_purple(robot, speed_level=[0.2, 0.25, 0.38, 0.4]):
    global SHOULD_RUN

    rospy.loginfo("starting task purple")

    do_generic(robot)

    if not SHOULD_RUN:
        robot.stop_motors()
        return

    robot.move_to(0.16943, 0.34233, -1.23446, target_speed=speed_level[2], precision=2, should_set_heading=False)
    robot.move_to(0.05432, 1.48142, -90.18769, target_speed=speed_level[2], should_avoid_obstacles=True)

    if not SHOULD_RUN:
        robot.stop_motors()
        return

    robot.push_left()
    d = robot.move_to_surface(target_speed=speed_level[0], access_key="back", move_timeout=7); rospy.sleep(0.5)

    robot.push_right(); rospy.sleep(0.5)
    robot.move_linear(d, target_speed=speed_level[0], should_avoid_obstacles=True, move_timeout=7, precision=2) # add should_avoid_obstacles

    rospy.loginfo("adding score for particle accelerator")
    robot.update_robot_score(10.0)

    if not SHOULD_RUN:
        robot.stop_motors()
        return

    robot.move_to(0.02273, 2.05537, 88.13082, target_speed=speed_level[2], should_avoid_obstacles=True)
    robot.open_gripper(); rospy.sleep(0.5)

    rospy.loginfo("adding score for flap open")
    robot.update_robot_score(10.0)

    if not SHOULD_RUN:
        robot.stop_motors()
        return

    d = robot.move_to_surface(target_speed=speed_level[0], move_timeout=8) # , move_timeout=8
    robot.close_gripper(); rospy.sleep(0.7)
    robot.move_linear(-d, target_speed=speed_level[0], move_timeout=8, should_avoid_obstacles=True) # , move_timeout=8

    rospy.loginfo("adding score for releasing goldium")
    robot.update_robot_score(20.0)

    if not SHOULD_RUN:
        robot.stop_motors()
        return

    robot.move_to(-0.8771, 1.1, -83.07387, target_speed=speed_level[2], should_avoid_obstacles=True)
    d = robot.move_to_surface(target_speed=speed_level[0], move_timeout=5)
    robot.open_gripper(); rospy.sleep(0.7)

    rospy.loginfo("adding score for goldium in weighing area")
    robot.update_robot_score(24.0)

    rospy.loginfo("done with task purple")

def table_1_yellow(robot, speed_level=[0.2, 0.25, 0.38, 0.4]):
    global SHOULD_RUN

    rospy.loginfo("starting task yellow")

    do_generic(robot)

    if not SHOULD_RUN:
        robot.stop_motors()
        return

    robot.move_to(-0.18233, 0.30964, -0.33545, target_speed=speed_level[2], precision=2, should_set_heading=False)
    robot.move_to(-0.0151, 1.48142, 88.16924, target_speed=speed_level[2], should_avoid_obstacles=True)

    if not SHOULD_RUN:
        robot.stop_motors()
        return

    robot.push_right()
    d = robot.move_to_surface(target_speed=speed_level[0], access_key="back", move_timeout=7); rospy.sleep(0.5)

    robot.push_left(); rospy.sleep(0.7)
    robot.move_linear(d, target_speed=speed_level[0], should_avoid_obstacles=True, move_timeout=7, precision=2) # add should_avoid_obstacles

    rospy.loginfo("adding score for particle accelerator")
    robot.update_robot_score(10.0)

    if not SHOULD_RUN:
        robot.stop_motors()
        return

    robot.move_to(-0.16383, 2.0498, -87.90616, target_speed=speed_level[2], should_avoid_obstacles=True)
    robot.open_gripper(); rospy.sleep(0.5)

    rospy.loginfo("adding score for flap open")
    robot.update_robot_score(10.0)

    if not SHOULD_RUN:
        robot.stop_motors()
        return

    d = robot.move_to_surface(target_speed=speed_level[0], move_timeout=8)
    robot.close_gripper(); rospy.sleep(0.7)
    robot.move_linear(-d, target_speed=speed_level[0], should_avoid_obstacles=True, move_timeout=8)

    rospy.loginfo("adding score for releasing goldium")
    robot.update_robot_score(20.0)

    if not SHOULD_RUN:
        robot.stop_motors()
        return

    robot.move_to(0.41827, 1.04506, 77.68496, target_speed=speed_level[2], should_avoid_obstacles=True, precision=2)
    robot.move_to(0.87744, 1.14051, 77.43708, target_speed=speed_level[2])
    d = robot.move_to_surface(target_speed=speed_level[0], move_timeout=5)
    robot.open_gripper(); rospy.sleep(1.0)

    rospy.loginfo("adding score for goldium in weighing area")
    robot.update_robot_score(24.0)

    rospy.loginfo("done with yellow")


rospy.init_node("eurobot_task_handler", anonymous=False)
rospy.loginfo("initializing the robot...")

robot = AdvancedRobotInterface()
robot.initialize()

def eurobot_task_cmd_handler(msg):
    global robot, SHOULD_RUN

    command = msg.data
    rospy.loginfo("[INFO] eurobot task received the command: {}".format(command))

    if command == "reset":
        rospy.loginfo("[INFO] eurobot task received command to reset")
        
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
    
    elif command == "start_yellow":
        rospy.loginfo("[INFO] eurobot task received command to start yellow")

        if not SHOULD_RUN:
            SHOULD_RUN = True
            thread.start_new_thread(table_1_yellow, (robot,))
            rospy.loginfo("[INFO] started yellow task")
        else:
            rospy.loginfo("task is already running")

    elif command == "start_purple":
        rospy.loginfo("[INFO] eurobot task received command to start purple")

        if not SHOULD_RUN:
            SHOULD_RUN = True
            thread.start_new_thread(table_1_purple, (robot,))
            rospy.loginfo("[INFO] started purple task")
        else:
            rospy.loginfo("task is already running")

    elif command == "kill_task":
        SHOULD_RUN = False

rospy.Subscriber('eurobot_task_cmd', String, eurobot_task_cmd_handler)

rospy.loginfo("ctrl-c to terminate")
rospy.spin()

rospy.loginfo("terminating....")
robot.terminate()
