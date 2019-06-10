#!/usr/bin/env python
from __future__ import division
import thread
import rospy, signal
from std_msgs.msg import String
from robot_interface_advanced import AdvancedRobotInterface

x = raw_input('Please choose(Square:s, Isosceles Triangle:ts, Equilateral Triangle:te): ')
x = str(x)
print("You've chosen option %s" % x)

# ----------------------------------------------------------------------------------------------------------------------
# Square (Which can have different possibilities of the movement from the starting point) <--- This needs to be done
if x == 's' or 'S':
  rospy.init_node("eurobot_task_handler", anonymous=False)
  rospy.loginfo("initializing the robot...")
  robot = AdvancedRobotInterface()
  robot.initialize()
  rospy.loginfo("waiting for pull to start")
  robot.wait_for_pull_to_start(state=True);
  rospy.sleep(3.0)

  def drive_square(dist):
    # opt = raw_input('Please select option (1,2,3,4,5,6,7,8) of square alignment: ')
    # opt = int(opt)

     robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.12)
     robot.move_angular(90)
     robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.12)
     robot.move_angular(90)
     robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.12)
     robot.move_angular(90)
     robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.12)
     robot.move_angular(90)

  drive_square(0.5)
# ----------------------------------------------------------------------------------------------------------------------
# Isosceles Triangle (Which can have different possibilities of the movement from the starting point)<--Needs to be done
elif x == 'ts' or 'TS':
  rospy.init_node("eurobot_task_handler", anonymous=False)
  rospy.loginfo("initializing the robot...")
  robot = AdvancedRobotInterface()
  robot.initialize()
  rospy.loginfo("waiting for pull to start")
  robot.wait_for_pull_to_start(state=True);
  rospy.sleep(3.0)

  def drive_triangle_isosceles(dist):
    #opt = raw_input('Please select option (1,2,3,4,5,6,7,8) of triangle alignment: ')
    #opt = int(opt)

    robot.move_angular(30)
    robot.move_linear(dist , should_avoid_obstacles=True, obstacle_backup_distance= 0.1, clearing_distance= 0.1)
    robot.move_angular(120)
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.1)
    robot.move_angular(120)
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.1)
    robot.move_angular(120)

    robot.move_angular(30)
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.1)
    robot.move_angular(125)
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.1)
    robot.move_angular(120)
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.1)

    robot.move_angular(125)
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.1)
    robot.move_angular(125)
    robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.1)


  drive_triangle_isosceles(0.75)

# ----------------------------------------------------------------------------------------------------------------------
# Equilateral Triangle(Which can have different possibilities of the movement from the starting point)<-Needs to be done

elif x == 'te' or 'TE':
  rospy.init_node("eurobot_task_handler", anonymous=False)
  rospy.loginfo("initializing the robot...")
  robot = AdvancedRobotInterface()
  robot.initialize()
  rospy.loginfo("waiting for pull to start")
  robot.wait_for_pull_to_start(state=True);
  rospy.sleep(3.0)

  def drive_triangle_equilateral(dist):
   # opt = raw_input('Please select option (1,2,3,4,5,6,7,8) of triangle alignment: ')
   # opt = int(opt)
   robot.move_angular(30)
   robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.1)
   robot.move_angular(120)
   robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.1)
   robot.move_angular(30)
   robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.1)
   robot.move_angular(120)
   robot.move_linear(dist, should_avoid_obstacles=True, obstacle_backup_distance=0.1, clearing_distance=0.1)

  drive_triangle_equilateral(0.5)

# --------------------------------------------------------------------------------------------------------------------
rospy.loginfo("ctrl-c to terminate")
rospy.spin()

rospy.loginfo("terminating....")
robot.terminate()

